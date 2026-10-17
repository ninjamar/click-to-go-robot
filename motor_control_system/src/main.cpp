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
CommandPacket queue[QUEUE_SIZE]; // NOLINT(modernize-avoid-c-arrays)
// Circular buffer; avoids shifting every element on pop.
uint8_t queueHead = 0; // index of oldest item
uint8_t queueCount = 0; // number of items in queue
// tail = index where new item should be added

// Currently executing command
bool isActive = false;
CommandPacket activeCommand;
unsigned long activeStartTime = 0;
unsigned long lastTickTime = 0;

// Per-wheel progress toward activeCommand.arg for KIND_DISTANCE
// (MOVE_DIST/ROTATE_ANGLE); each wheel is tracked and driven independently.
float accumulatedM_A = 0.0f;
float accumulatedM_B = 0.0f;

// Progress for KIND_SHAFT_ROTATION (DEBUG_RUN_N_ROTATIONS) only.
float accumulatedRad = 0.0f;

// Stall watchdog state - last progress value seen, and when.
float lastProgressValue = 0.0f;
unsigned long lastProgressTime = 0;

// Persistent signed (direction) odometry
float linearDisplacementA = 0.0f;
float linearDisplacementB = 0.0f;

float linearDistanceA = 0.0f; // magnitude
float linearDistanceB = 0.0f;

// Deferred OP_POLL_W/OP_POLL_DISTANCE request; single slot, most recent
// overwrites
bool pollPending = false;
CommandPacket pollRequest;

void setup() {
    Serial.begin(BAUD_RATE);
    odometerDriver.begin();
}

// Classifies the active command's progress-tracking style for the control tick below.
enum ActiveOpKind : uint8_t {
    KIND_TIME_MOVE, // OP_QUEUE_MOVE / OP_QUEUE_ROTATE - duration-based
    KIND_DISTANCE, // OP_QUEUE_MOVE_DIST / OP_QUEUE_ROTATE_ANGLE - wheel-geometry-based
    KIND_SHAFT_ROTATION, // OP_DEBUG_RUN_N_ROTATIONS - raw motor shaft angle
    KIND_RAW_PWM // OP_DEBUG_SET_PWM - no progress metric, runs until stopped
};

ActiveOpKind classifyOp(uint8_t op) {
    switch (op) {
        case OP_QUEUE_MOVE_DIST:
        case OP_QUEUE_ROTATE_ANGLE:
            return KIND_DISTANCE;
        case OP_DEBUG_RUN_N_ROTATIONS:
            return KIND_SHAFT_ROTATION;
        case OP_DEBUG_SET_PWM:
            return KIND_RAW_PWM;
        default: // OP_QUEUE_MOVE, OP_QUEUE_ROTATE
            return KIND_TIME_MOVE;
    }
}

// v = sqrt(2*a*d); min of accel/decel phases, capped at MAX_ANGULAR_VEL.
float computeTargetOmegaByDistance(float traveledShaftRad, float remainingShaftRad) {
    float omegaUp = sqrtf(2.0f * MAX_ANGULAR_ACCEL * max(traveledShaftRad, 0.0f));
    float omegaDown = sqrtf(2.0f * MAX_ANGULAR_ACCEL * max(remainingShaftRad, 0.0f));
    return min(min(omegaUp, omegaDown), MAX_ANGULAR_VEL);
}

// Same trapezoid, parameterized by time (no distance target available).
float computeTargetOmegaByTime(float elapsedSec, float remainingSec) {
    float omegaUp = MAX_ANGULAR_ACCEL * elapsedSec;
    float omegaDown = MAX_ANGULAR_ACCEL * max(remainingSec, 0.0f);
    return min(min(omegaUp, omegaDown), MAX_ANGULAR_VEL);
}

uint8_t pwmFromOmega(float omega) {
    float pwm = MIN_PWM + PWM_PER_RAD_S * max(omega, 0.0f);
    return static_cast<uint8_t>(constrain(pwm, MIN_PWM, MAX_PWM));
}

// All poll calls pass through here. Only accumulates distance if the direction
// is known (via isActive)
void pollAndAccumulate(float& w1, float& w2) {
    float dtSec;
    odometerDriver.poll(w1, w2, dtSec);

    float x1 = w1 * dtSec * WHEEL_RADIUS_M / GEAR_RATIO;
    float x2 = w2 * dtSec * WHEEL_RADIUS_M / GEAR_RATIO;
    linearDistanceA += x1;
    linearDistanceB += x2;

    if (isActive) {
        auto sign = static_cast<float>(activeCommand.direction);

        linearDisplacementA += sign * x1;
        linearDisplacementB += sign * x2;
    }
}

void sendVelocityResponse(const CommandPacket& pk, ResponseStatus status) {
    float w1;
    float w2;
    pollAndAccumulate(w1, w2);
    // Direction is only known while a command is actively driving; otherwise
    // report magnitude; see isActive
    float sign = isActive ? static_cast<float>(activeCommand.direction) : 1.0f;
    w1 *= sign;
    w2 *= sign;
    float v1 = w1 * WHEEL_RADIUS_M / GEAR_RATIO;
    float v2 = w2 * WHEEL_RADIUS_M / GEAR_RATIO;
    serialProtocol.sendResponse(pk, status, isActive, w1, w2, v1, v2, linearDistanceA, linearDistanceB, linearDisplacementA, linearDisplacementB);
}

// DEBUG: periodic pulse counts, to check whether motor A's live CH_A drops
// out in one direction. Remove once the encoder issue is resolved.
// void printDebugCounts() {
//     static unsigned long lastDebugPrint = 0;
//     unsigned long now = millis();
//     if (now - lastDebugPrint < 250) {
//         return;
//     }
//     lastDebugPrint = now;

//     unsigned long c1;
//     unsigned long c2;
//     odometerDriver.debugCounts(c1, c2);
//     Serial.print("encA chA=");
//     Serial.print(c1);
//     Serial.print("  encB chA=");
//     Serial.println(c2);
// }

void loop() {
    // printDebugCounts();

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
            case OP_POLL_DIST:
                if (!isActive) {
                    sendVelocityResponse(packet, STATUS_SUCCESS);
                } else {
                    pollPending = true;
                    pollRequest = packet;
                }
                break;
            case OP_RESET_DIST:
                linearDisplacementA = 0.0f;
                linearDisplacementB = 0.0f;
                linearDistanceA = 0.0f;
                linearDistanceB = 0.0f;

                serialProtocol.sendStatus(packet, STATUS_SUCCESS);
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

        ActiveOpKind kind = classifyOp(activeCommand.operation);

        // Call on every tick for distance
        float w1;
        float w2;
        pollAndAccumulate(w1, w2);

        bool done = false;
        ResponseStatus doneStatus = STATUS_SUCCESS;
        float remaining = 0.0f;
        float shaftRadPerUnit = 1.0f; // unused for KIND_RAW_PWM/KIND_TIME_MOVE

        if (kind == KIND_RAW_PWM) {
            // No progress metric - this op only ends via OP_STOP or this watchdog.
            done = elapsedMs > MAX_COMMAND_MS;
            doneStatus = STATUS_TIMEOUT;
        } else if (kind == KIND_DISTANCE || kind == KIND_SHAFT_ROTATION) {
            float dt = MS_TO_S(CONTROL_TICK_MS);

            if (kind == KIND_SHAFT_ROTATION) {
                // Raw shaft angle; independent of emperical constants
                accumulatedRad += fabsf(w2) * dt;

                float targetRad = activeCommand.arg * (2 * PI);
                remaining = targetRad - accumulatedRad;
                shaftRadPerUnit = 1.0f;
                done = accumulatedRad >= targetRad;
            } else {
                float dsA = w1 * dt / GEAR_RATIO * WHEEL_RADIUS_M;
                float dsB = w2 * dt / GEAR_RATIO * WHEEL_RADIUS_M;

                // MOVE_DIST: each wheel targets the commanded distance directly.
                // ROTATE_ANGLE: each wheel targets the arc length it must cover
                // for the commanded chassis rotation (opposite directions, equal
                // magnitude), rather than the old signed (dsB-dsA) difference -
                // this lets each wheel be driven and stopped independently.
                float target = (activeCommand.operation == OP_QUEUE_MOVE_DIST)
                    ? activeCommand.arg
                    : activeCommand.arg * TRACK_WIDTH_M / 2.0f;

                accumulatedM_A += fabsf(dsA);
                accumulatedM_B += fabsf(dsB);

                float remainingA = target - accumulatedM_A;
                float remainingB = target - accumulatedM_B;
                remaining = target - (accumulatedM_A + accumulatedM_B) / 2.0f; // feeds the shared ramp-down only
                shaftRadPerUnit = GEAR_RATIO / WHEEL_RADIUS_M;
                done = max(remainingA, remainingB) <= 0.0f;
            }

            if (!done) {
                if (elapsedMs > MAX_COMMAND_MS) {
                    done = true;
                    doneStatus = STATUS_TIMEOUT;
                } else {
                    // MOVE_DIST/ROTATE_ANGLE track the slower wheel, so a
                    // stalled wheel trips this even if its partner is still
                    // moving; DEBUG_RUN_N_ROTATIONS tracks accumulatedRad.
                    bool isDistanceKind = activeCommand.operation == OP_QUEUE_MOVE_DIST
                        || activeCommand.operation == OP_QUEUE_ROTATE_ANGLE;
                    float progress = isDistanceKind ? min(accumulatedM_A, accumulatedM_B) : accumulatedRad;
                    if (progress > lastProgressValue) {
                        lastProgressValue = progress;
                        lastProgressTime = now;
                    } else if (now - lastProgressTime > STALL_TIMEOUT_MS) {
                        done = true;
                        doneStatus = STATUS_TIMEOUT;
                    }
                }
            }
        } else { // KIND_TIME_MOVE: OP_QUEUE_MOVE or OP_QUEUE_ROTATE
            float totalMs = S_TO_MS(activeCommand.arg);
            remaining = totalMs - static_cast<float>(elapsedMs);
            done = elapsedMs >= static_cast<unsigned long>(totalMs);
        }

        if (done) {
            motorDriver.stop();
            serialProtocol.sendStatus(activeCommand, doneStatus);
            isActive = false;
        } else if (kind == KIND_RAW_PWM) {
            auto pwm = static_cast<uint8_t>(constrain(activeCommand.arg, 0.0f, MAX_PWM));
            motorDriver.driveMove(activeCommand.direction, pwm, pwm);
        } else if (kind == KIND_DISTANCE) {
            float traveled = (accumulatedM_A + accumulatedM_B) / 2.0f;
            float omegaRef = computeTargetOmegaByDistance(traveled * shaftRadPerUnit, max(remaining, 0.0f) * shaftRadPerUnit);
            uint8_t pwmRef = pwmFromOmega(omegaRef);

            // B ahead of A -> positive lag -> speed A up, slow B down.
            float lagA = accumulatedM_B - accumulatedM_A;
            auto pwmA = static_cast<uint8_t>(constrain(pwmRef + SYNC_KP_PWM_PER_M * lagA, MIN_PWM, MAX_PWM));
            auto pwmB = static_cast<uint8_t>(constrain(pwmRef - SYNC_KP_PWM_PER_M * lagA, MIN_PWM, MAX_PWM));

            if (activeCommand.operation == OP_QUEUE_MOVE_DIST) {
                motorDriver.driveMove(activeCommand.direction, pwmA, pwmB);
            } else {
                motorDriver.driveRotate(activeCommand.direction, pwmA, pwmB);
            }
        } else {
            uint8_t pwm;
            if (kind == KIND_TIME_MOVE) {
                pwm = pwmFromOmega(computeTargetOmegaByTime(MS_TO_S(elapsedMs), MS_TO_S(max(remaining, 0.0f))));
            } else { // KIND_SHAFT_ROTATION
                pwm = pwmFromOmega(computeTargetOmegaByDistance(accumulatedRad * shaftRadPerUnit, max(remaining, 0.0f) * shaftRadPerUnit));
            }
            if (activeCommand.operation == OP_QUEUE_MOVE || activeCommand.operation == OP_DEBUG_RUN_N_ROTATIONS) {
                motorDriver.driveMove(activeCommand.direction, pwm, pwm);
            } else {
                motorDriver.driveRotate(activeCommand.direction, pwm, pwm);
            }
        }
    }

    // Answer a deferred poll before the next queued command dispatches.
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
        accumulatedM_A = 0.0f;
        accumulatedM_B = 0.0f;
        accumulatedRad = 0.0f;
        lastProgressValue = 0.0f;
        lastProgressTime = activeStartTime;
        isActive = true;
    }
}
