#include "config.hpp"
#include "motor_driver.hpp"
#include "serial_protocol.hpp"
#include <Arduino.h>

MotorDriver motorDriver(MOTOR_EN_PIN, MOTOR_IN1_PIN, MOTOR_IN2_PIN);

void setup()
{
    Serial.begin(BAUD_RATE);
}

void loop()
{
    CommandPacket cmd;
    if (readCommand(Serial, cmd)) {
        switch (cmd.operation) {
        case OP_MOVE:
            motorDriver.move(cmd.arg);
            break;
        case OP_ROTATE:
            motorDriver.rotate(cmd.arg);
            break;
        case OP_STOP:
            motorDriver.stop();
            break;
        }
        ResponsePacket resp = buildResponse(cmd.seq_id);
        sendResponse(Serial, resp);
    }
}