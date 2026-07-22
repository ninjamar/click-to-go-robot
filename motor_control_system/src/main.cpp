#include "config.hpp"
#include "macros.hpp"
#include "motor_driver.hpp"
#include "odometer_driver.hpp"
#include "serial_protocol.hpp"
#include <Arduino.h>
#include <math.h>

MotorDriver motorDriver(MOTOR_A_EN_PIN, MOTOR_A_IN1_PIN, MOTOR_A_IN2_PIN, MOTOR_B_EN_PIN, MOTOR_B_IN1_PIN, MOTOR_B_IN2_PIN);
OdometerDriver odometerDriver(ENCODER_MOTORA_CH_A_PIN, ENCODER_MOTORA_CH_B_PIN, ENCODER_MOTORB_CH_A_PIN, ENCODER_MOTORB_CH_B_PIN);
SerialProtocol serialProtocol(Serial);

// Command queue state
CommandPacket queue[QUEUE_SIZE];
// This is a circular buffer: the pop operation would require a shift of each item.
// So, a circular buffer is used.
uint8_t queueHead = 0; // index of oldest item
uint8_t queueCount = 0; // number of items in queue
// tail = index where new item should be added

// Currently executing command
bool isActive = false;
CommandPacket activeCommand;
unsigned long activeStartTime = 0;
unsigned long lastTickTime = 0;

// Progress accumulators for OP_QUEUE_MOVE_DIST / OP_QUEUE_ROTATE_ANGLE only —
// unsigned running totals of "how far/how much have I gone", regardless of
// direction, compared against activeCommand.arg (also unsigned magnitude).
float accumulatedM = 0.0f;
float accumulatedRad = 0.0f;

// Deferred OP_POLL_W request (single slot — a second poll while one is
// already pending simply overwrites it; last request's seq_id wins).
bool pollPending = false;
CommandPacket pollRequest;

void setup() {
    Serial.begin(BAUD_RATE);
    odometerDriver.begin();
}

bool isDistanceOp(uint8_t op) {
    return op == OP_QUEUE_MOVE_DIST || op == OP_QUEUE_ROTATE_ANGLE;
}

// Two independent fixed ramps — up from MIN_PWM over RAMP_UP_MS, down to
// MIN_PWM over the last rampDownAt (same units as remaining) — take the min.
// A move shorter than RAMP_UP_MS + rampDownAt never reaches MAX_PWM; no
// separate short-move case needed.
uint8_t computeRampedPwm(unsigned long elapsedMs, float remaining, float rampDownAt) {
    float pwmUp = MIN_PWM + (MAX_PWM - MIN_PWM) * min(elapsedMs / (float)RAMP_UP_MS, 1.0f);
    float pwmDown = MIN_PWM + (MAX_PWM - MIN_PWM) * min(remaining / rampDownAt, 1.0f);
    float pwm = min(pwmUp, pwmDown);
    if (pwm < MIN_PWM) {
        pwm = MIN_PWM;
    }
    return (uint8_t)pwm;
}

void sendVelocityResponse(const CommandPacket& pk, ResponseStatus status) {
    float w1;
    float w2;
    odometerDriver.poll(w1, w2);
    // The encoder only measures rotation magnitude reliably (see
    // odometer_driver.hpp) - direction comes from what we commanded, not
    // from the encoder itself.
    float sign = isActive ? (float)activeCommand.direction : 0.0f;
    w1 *= sign;
    w2 *= sign;
    float v1 = w1 * WHEEL_RADIUS_M / GEAR_RATIO;
    float v2 = w2 * WHEEL_RADIUS_M / GEAR_RATIO;
    serialProtocol.sendResponse(pk, status, w1, w2, v1, v2);
}

void loop() {
    // Intake
    CommandPacket packet;
    if (serialProtocol.readCommand(packet)) {
        switch (packet.operation) {
            // Handle non-queue operations
            case OP_STOP:
                motorDriver.stop();
                isActive = false;
                serialProtocol.sendStatus(packet, STATUS_SUCCESS);
                break;
            case OP_POLL_W:
                if (!isActive) {
                    sendVelocityResponse(packet, STATUS_SUCCESS);
                } else {
                    pollPending = true;
                    pollRequest = packet;
                }
                break;
            default:
                // Queued operations: add them to the queue
                uint8_t totalPending = queueCount + (isActive ? 1 : 0);
                if (totalPending < QUEUE_SIZE) {
                    uint8_t tail = (queueHead + queueCount) % QUEUE_SIZE;
                    queue[tail] = packet;
                    queueCount++;
                    serialProtocol.sendStatus(packet, STATUS_OK);
                } else {
                    serialProtocol.sendStatus(packet, STATUS_REJECTED_FULL);
                }
                break;
        }
    }

    // Control tick: drive the active command's motors at the ramped PWM and
    // check for completion. Runs at most once per CONTROL_TICK_MS.
    if (isActive && millis() - lastTickTime >= CONTROL_TICK_MS) {
        unsigned long now = millis();
        unsigned long elapsedMs = now - activeStartTime;
        lastTickTime = now;

        bool done = false;
        ResponseStatus doneStatus = STATUS_SUCCESS;
        float remaining;
        float rampDownAt;

        if (isDistanceOp(activeCommand.operation)) {
            float w1;
            float w2;
            odometerDriver.poll(w1, w2);
            float dt = MS_TO_S(CONTROL_TICK_MS);
            float dsA = w1 * dt / GEAR_RATIO * WHEEL_RADIUS_M;
            float dsB = w2 * dt / GEAR_RATIO * WHEEL_RADIUS_M;

            if (activeCommand.operation == OP_QUEUE_MOVE_DIST) {
                accumulatedM += fabs((dsA + dsB) / 2.0f);
                remaining = activeCommand.arg - accumulatedM;
                rampDownAt = RAMP_DOWN_DISTANCE_M;
                done = accumulatedM >= activeCommand.arg;
            } else { // OP_QUEUE_ROTATE_ANGLE
                accumulatedRad += fabs((dsB - dsA) / TRACK_WIDTH_M);
                remaining = activeCommand.arg - accumulatedRad;
                rampDownAt = RAMP_DOWN_ANGLE_RAD;
                done = accumulatedRad >= activeCommand.arg;
            }

            if (!done && elapsedMs > MAX_COMMAND_MS) {
                done = true;
                doneStatus = STATUS_TIMEOUT;
            }
        } else { // OP_QUEUE_MOVE or OP_QUEUE_ROTATE
            float totalMs = S_TO_MS(activeCommand.arg);
            remaining = totalMs - elapsedMs;
            rampDownAt = (float)RAMP_DOWN_MS;
            done = elapsedMs >= (unsigned long)totalMs;
        }

        if (done) {
            motorDriver.stop();
            serialProtocol.sendStatus(activeCommand, doneStatus);
            isActive = false;
        } else {
            uint8_t pwm = computeRampedPwm(elapsedMs, max(remaining, 0.0f), rampDownAt);
            if (activeCommand.operation == OP_QUEUE_MOVE || activeCommand.operation == OP_QUEUE_MOVE_DIST) {
                motorDriver.driveMove(activeCommand.direction, pwm);
            } else {
                motorDriver.driveRotate(activeCommand.direction, pwm);
            }
        }
    }

    // Poll (before the next queue command dispatches, so a poll is fulfiled before)
    if (pollPending) {
        sendVelocityResponse(pollRequest, STATUS_SUCCESS);
        pollPending = false;
    }

    // Dispatch: if nothing is active and queue has items, pop front and
    // reset progress tracking. Driving starts on the next control tick.
    if (!isActive && queueCount > 0) {
        activeCommand = queue[queueHead];
        queueHead = (queueHead + 1) % QUEUE_SIZE;
        queueCount--;
        activeStartTime = millis();
        lastTickTime = activeStartTime - CONTROL_TICK_MS; // fire the first tick immediately
        accumulatedM = 0.0f;
        accumulatedRad = 0.0f;
        isActive = true;
    }
}
