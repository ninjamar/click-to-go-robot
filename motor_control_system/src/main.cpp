#include <Arduino.h>
#include "serial_protocol.hpp"

void setup() {
    Serial.begin(57600);
}

void loop() {
    CommandPacket cmd;
    if (readCommand(Serial, cmd)) {
        // Echo
        ResponsePacket resp = buildResponse(cmd.seq_id);
        sendResponse(Serial, resp);
    }
}