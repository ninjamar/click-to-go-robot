#pragma once

#include <Arduino.h>

// SI units
enum CommandOperation {
    // arg: duration(s)
    OP_QUEUE_MOVE = 1, // dir: back(-1)/stop(0)/fwd(1)
    OP_QUEUE_ROTATE = 2, // dir: cw(-1)/stop(0)/ccw(1)
    // arg: none
    OP_STOP = 3,
    OP_POLL_W = 4,
    // arg: distance(m)
    OP_QUEUE_MOVE_DIST = 5, // dir: back(-1)/stop(0)/fwd(1)
    // arg: angle(rad)
    OP_QUEUE_ROTATE_ANGLE = 6, // dir: cw(-1)/stop(0)/ccw(1)
    // arg: integers, no si units
    OP_DEBUG_RUN_N_ROTATIONS = 7,
    OP_DEBUG_SET_PWM = 8,
    // arg: none
    OP_RESET_DISTANCE = 9,
    OP_POLL_DISTANCE = 10
};

enum ResponseStatus {
    STATUS_OK = 0,
    STATUS_SUCCESS = 1,
    STATUS_REJECTED_FULL = 2,
    STATUS_TIMEOUT = 3,
};

struct CommandPacket {
    uint8_t seq_id;
    uint8_t operation;
    float arg; // meaning depends on operation, see above
    int8_t direction; // -1, 0, or 1; back, stop, forward
    // uint8_t checksum;
} __attribute__((packed)); // Pack it tightly to match python struct

struct ResponsePacket {
    uint8_t seq_id;
    uint8_t status; // ResponseStatus
    uint8_t is_active; // if 0, w1/w2/v1/v2 are unsigned magnitude only (no known direction)
    float w1; // angular velocity of motor A's raw output shaft, rad/s
    float w2; // angular velocity of motor B's raw output shaft, rad/s
    float v1; // linear velocity of wheel A, m/s
    float v2; // linear velocity of wheel B, m/s
    float d1; // cumulative signed linear distance of wheel A since last OP_RESET_DISTANCE, m
    float d2; // cumulative signed linear distance of wheel B since last OP_RESET_DISTANCE, m
    // uint8_t checksum;
} __attribute__((packed));

class SerialProtocol {
public:
    SerialProtocol(Stream& serial)
        : _serial(serial) {
    }

    bool readCommand(CommandPacket& out) {
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

    void sendResponse(const CommandPacket& pk, ResponseStatus status, bool isActive, float w1, float w2, float v1, float v2, float d1, float d2) {
        sendResponsePacket(buildResponse(pk, status, isActive, w1, w2, v1, v2, d1, d2));
    }

    void sendStatus(const CommandPacket& pk, ResponseStatus status) {
        sendResponse(pk, status, false, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }

private:
    Stream& _serial;

    ResponsePacket buildResponse(const CommandPacket& in, uint8_t status, bool isActive, float w1, float w2, float v1, float v2, float d1, float d2) {
        ResponsePacket resp;
        resp.seq_id = in.seq_id;
        resp.status = status;
        resp.is_active = isActive ? 1 : 0;
        resp.w1 = w1;
        resp.w2 = w2;
        resp.v1 = v1;
        resp.v2 = v2;
        resp.d1 = d1;
        resp.d2 = d2;
        return resp;
    }

    void sendResponsePacket(const ResponsePacket& packet) {
        _serial.write(
            reinterpret_cast<const uint8_t*>(&packet),
            sizeof(ResponsePacket));
    }
};
