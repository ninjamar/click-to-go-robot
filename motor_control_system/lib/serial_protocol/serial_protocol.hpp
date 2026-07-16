#pragma once

#include <Arduino.h>

enum CommandOperation {
    OP_MOVE = 1,
    OP_ROTATE = 2,
    OP_STOP = 3
};

enum ResponseStatus {
    STATUS_OK = 0,
    STATUS_SUCCESS = 1,
    STATUS_REJECTED_FULL = 2,
};

struct CommandPacket {
    uint8_t seq_id;
    uint8_t operation; // Operation
    uint64_t arg;
    // uint8_t checksum;
} __attribute__((packed)); // Pack it tightly to match Python.

struct ResponsePacket {
    uint8_t seq_id;
    uint8_t status; // ResponseStatus
    // uint8_t checksum;
} __attribute__((packed));

// bool checkCommandValidity(const CommandPacket& packet)
// {
//     uint8_t expected = packet.seq_id + packet.operation + packet.arg;
//     return packet.checksum == expected;
// }

ResponsePacket buildResponse(const CommandPacket& in, uint8_t status)
{
    ResponsePacket resp;
    resp.seq_id = in.seq_id;
    resp.status = status;
    // resp.checksum = seq_id;
    return resp;
}

bool readCommand(Stream& serial, CommandPacket& out)
{
    if (serial.available() < static_cast<int>(sizeof(CommandPacket))) { // cast size_t to int
        return false;
    }

    size_t bytesRead = serial.readBytes(
        reinterpret_cast<char*>(&out),
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

void sendStatus(const CommandPacket& pk, ResponseStatus status)
{
    sendResponse(Serial, buildResponse(pk, status));
}