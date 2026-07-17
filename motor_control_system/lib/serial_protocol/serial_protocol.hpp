#pragma once

#include <Arduino.h>

enum CommandOperation {
    OP_QUEUE_MOVE = 1,
    OP_QUEUE_ROTATE = 2,
    OP_STOP = 3,
    OP_POLL_W = 4
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
    int8_t direction; // -1, 0, or 1
    // uint8_t checksum;
} __attribute__((packed)); // Pack it tightly to match Python.

struct ResponsePacket {
    uint8_t seq_id;
    uint8_t status; // ResponseStatus
    float w1; // angular velocity of motor A's raw output shaft, rad/s
    float w2; // angular velocity of motor B's raw output shaft, rad/s
    // uint8_t checksum;
} __attribute__((packed));

class SerialProtocol {
public:
    SerialProtocol(Stream& serial)
        : _serial(serial)
    {
    }

    bool readCommand(CommandPacket& out)
    {
        if (_serial.available() < static_cast<int>(sizeof(CommandPacket))) {
            return false;
        }

        size_t bytesRead = _serial.readBytes(
            reinterpret_cast<char*>(&out),
            sizeof(CommandPacket));

        if (bytesRead != sizeof(CommandPacket)) {
            return false;
        }

        return true;
    }

    void sendResponse(const CommandPacket& pk, ResponseStatus status, float w1, float w2)
    {
        sendResponsePacket(buildResponse(pk, status, w1, w2));
    }

    void sendStatus(const CommandPacket& pk, ResponseStatus status)
    {
        sendResponse(pk, status, 0.0f, 0.0f); // 0.0 is double. 0.0f is float
    }

private:
    Stream& _serial;

    ResponsePacket buildResponse(const CommandPacket& in, uint8_t status, float w1 = 0.0f, float w2 = 0.0f)
    {
        ResponsePacket resp;
        resp.seq_id = in.seq_id;
        resp.status = status;
        resp.w1 = w1;
        resp.w2 = w2;
        return resp;
    }

    void sendResponsePacket(const ResponsePacket& packet)
    {
        _serial.write(
            reinterpret_cast<const uint8_t*>(&packet),
            sizeof(ResponsePacket));
    }
};