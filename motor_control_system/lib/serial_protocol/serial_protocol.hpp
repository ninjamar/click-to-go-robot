#pragma once

#include <Arduino.h>

enum Operation {
    OP_MOVE = 1,
    OP_ROTATE = 2,
    OP_STOP = 3
};

struct CommandPacket {
    uint8_t seq_id;
    uint8_t operation;
    uint64_t arg;
    // uint8_t checksum;
} __attribute__((packed)); // Pack it tightly to match Python.

struct ResponsePacket {
    uint8_t seq_id;
    // uint8_t checksum;
} __attribute__((packed));

// bool checkCommandValidity(const CommandPacket& packet)
// {
//     uint8_t expected = packet.seq_id + packet.operation + packet.arg;
//     return packet.checksum == expected;
// }

ResponsePacket buildResponse(uint8_t seq_id)
{
    ResponsePacket resp;
    resp.seq_id = seq_id;
    // resp.checksum = seq_id;
    return resp;
}

bool readCommand(Stream& serial, CommandPacket& outPacket)
{
    if (serial.available() < static_cast<int>(sizeof(CommandPacket))) { // cast size_t to int
        return false;
    }

    size_t bytesRead = serial.readBytes(
        reinterpret_cast<char*>(&outPacket),
        sizeof(CommandPacket));

    if (bytesRead != sizeof(CommandPacket)) {
        return false;
    }

    return true;
    // return checkCommandValidity(outPacket);
}

void sendResponse(Stream& serial, const ResponsePacket& packet)
{
    serial.write(
        reinterpret_cast<const uint8_t*>(&packet), // need to cast from packet to bitstream
        sizeof(ResponsePacket));
}