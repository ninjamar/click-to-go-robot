#pragma once

#include <Arduino.h>

// CommandPacket.arg is a float, in the natural SI unit for the operation:
//   OP_QUEUE_MOVE / OP_QUEUE_ROTATE   -> duration, seconds
//   OP_QUEUE_MOVE_DIST                -> distance, meters
//   OP_QUEUE_ROTATE_ANGLE             -> angle, radians
//   OP_DEBUG_RUN_N_ROTATIONS          -> number of raw motor-shaft rotations (e.g. 2.5).
//   OP_DEBUG_SET_PWM                  -> raw PWM duty cycle, 0-255. No ramp. Holds until
//                                         OP_STOP or MAX_COMMAND_MS elapses.
//   OP_STOP / OP_POLL_W / OP_RESET_DISTANCE / OP_POLL_DISTANCE -> unused (0)
//
// CommandPacket.direction meaning depends on the operation too:
//   OP_QUEUE_MOVE / OP_QUEUE_MOVE_DIST      -> forward(+1) / backward(-1) / stop(0)
//   OP_QUEUE_ROTATE / OP_QUEUE_ROTATE_ANGLE -> CCW(+1) / CW(-1) / stop(0)
//   OP_DEBUG_RUN_N_ROTATIONS                -> forward(+1) / backward(-1) / stop(0) of the
//                                               motor shaft (same convention as OP_QUEUE_MOVE -
//                                               this is a single-shaft test, not a chassis turn)
//   OP_DEBUG_SET_PWM                        -> forward(+1) / backward(-1); direction=0 stops
//                                               the motor but the command stays "active" (holding
//                                               a 0-duty stop) until OP_STOP/timeout - send
//                                               OP_STOP instead if you want to fully release it
enum CommandOperation {
    OP_QUEUE_MOVE = 1,
    OP_QUEUE_ROTATE = 2,
    OP_STOP = 3,
    OP_POLL_W = 4,
    OP_QUEUE_MOVE_DIST = 5,
    OP_QUEUE_ROTATE_ANGLE = 6,
    OP_DEBUG_RUN_N_ROTATIONS = 7,
    OP_DEBUG_SET_PWM = 8,
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
    uint8_t operation; // Operation
    float arg; // meaning depends on operation, see above
    int8_t direction; // -1, 0, or 1
    // uint8_t checksum;
} __attribute__((packed)); // Pack it tightly to match Python.

struct ResponsePacket {
    uint8_t seq_id;
    uint8_t status; // ResponseStatus
    uint8_t is_active; // whether a command is currently driving the motors - if 0, w1/w2/v1/v2 are unsigned magnitude only (no known direction)
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
