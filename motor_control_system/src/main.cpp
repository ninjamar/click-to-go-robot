#include "config.hpp"
#include "motor_driver.hpp"
#include "odometer_driver.hpp"
#include "serial_protocol.hpp"
#include <Arduino.h>

MotorDriver motorDriver(MOTOR_A_IN1_PIN, MOTOR_A_IN2_PIN, MOTOR_B_IN1_PIN, MOTOR_B_IN2_PIN);
OdometerDriver odometerDriver(ENCODER_A1_PIN, ENCODER_B1_PIN, ENCODER_A2_PIN, ENCODER_B2_PIN);
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
bool activeStarted = false;

// Deferred OP_POLL_W request (single slot — a second poll while one is
// already pending simply overwrites it; last request's seq_id wins).
bool pollPending = false;
CommandPacket pollRequest;

void setup() {
    Serial.begin(BAUD_RATE);
    odometerDriver.begin();
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
                    float w1;
                    float w2;
                    odometerDriver.poll(w1, w2);
                    serialProtocol.sendResponse(packet, STATUS_SUCCESS, w1, w2);
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

    // Check if the current command has finished; if it has, send response back.
    if (isActive && millis() - activeStartTime >= activeCommand.arg) {
        motorDriver.stop();
        motorDriver.getMotors().reset();
        serialProtocol.sendStatus(activeCommand, STATUS_SUCCESS);
        isActive = false;
    }

    // Poll (before the next queue command dispatches, so a poll is fulfiled before)
    if (pollPending) {
        float w1;
        float w2;
        odometerDriver.poll(w1, w2);
        serialProtocol.sendResponse(pollRequest, STATUS_SUCCESS, w1, w2);
        pollPending = false;
    }

    // Dispatch: if nothing is active and queue has items, pop front
    if (!isActive && queueCount > 0) {
        activeCommand = queue[queueHead];
        queueHead = (queueHead + 1) % QUEUE_SIZE;
        queueCount--;
        activeStartTime = millis();
        isActive = true;
        activeStarted = false;
    }

    // Start the active command
    if (isActive && !activeStarted) {
        switch (activeCommand.operation) {
            case OP_QUEUE_MOVE:
                motorDriver.move(activeCommand.arg, activeCommand.direction);
                break;
            case OP_QUEUE_ROTATE:
                motorDriver.rotate(activeCommand.arg, activeCommand.direction);
                break;
        }
        activeStarted = true;
    }
}