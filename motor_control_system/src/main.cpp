#include "config.hpp"
#include "motor_driver.hpp"
#include "serial_protocol.hpp"
#include <Arduino.h>

MotorDriver motorDriver(MOTOR_IN1_PIN, MOTOR_IN2_PIN); // no en pin

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

void setup()
{
    Serial.begin(BAUD_RATE);
}

void loop()
{
    // Intake, read command, check STOP, add to queue
    CommandPacket packet;
    if (readCommand(Serial, packet)) {
        if (packet.operation == OP_STOP) {
            // STOP bypasses queue: execute immediately and respond
            motorDriver.stop();
            isActive = false;
            sendStatus(packet, STATUS_SUCCESS);
        } else {
            // Add items to queue
            uint8_t totalPending = queueCount + (isActive ? 1 : 0); // 4 = queue + active
            if (totalPending < QUEUE_SIZE) {
                uint8_t tail = (queueHead + queueCount) % QUEUE_SIZE;
                queue[tail] = packet;
                queueCount++;
                sendStatus(packet, STATUS_OK);
            } else {
                // Queue full: reject
                sendStatus(packet, STATUS_REJECTED_FULL);
            }
        }
    }

    // Dispatch: if nothing is active and queue is non-empty, pop front and start job
    if (!isActive && queueCount > 0) {
        activeCommand = queue[queueHead];
        // move queue head to the right by one and wrap around
        queueHead = (queueHead + 1) % QUEUE_SIZE;
        queueCount--;
        activeStartTime = millis();
        isActive = true;
    }

    // Poll: if a command is active, drive it toward completion
    if (isActive) {
        switch (activeCommand.operation) {
        case OP_MOVE:
            motorDriver.move(activeCommand.arg);
        case OP_ROTATE:
            motorDriver.rotate(activeCommand.arg);
        }

        // Check duration
        if (millis() - activeStartTime >= activeCommand.arg) {
            motorDriver.getMotor().reset(); // Clean up library state
            sendStatus(activeCommand, STATUS_SUCCESS);
            isActive = false;
        }
    }
}