#pragma once
#include <L298N.h>
#include <L298NX2.h>

class MotorDriver {
public:
    MotorDriver(uint8_t pinIN1_A, uint8_t pinIN2_A, uint8_t pinIN1_B, uint8_t pinIN2_B)
        : _motors(pinIN1_A, pinIN2_A, pinIN1_B, pinIN2_B) {
    }

    void move(uint64_t duration, int8_t direction) {
        L298N::Direction dir = direction > 0
            ? L298N::FORWARD
            : (direction < 0 ? L298N::BACKWARD : L298N::STOP);
        _motors.runFor(duration, dir);
    }

    void rotate(uint64_t duration, int8_t direction) {
    }

    void stop() {
        _motors.stop();
    }

    L298NX2& getMotors() {
        return _motors;
    }

private:
    L298NX2 _motors;
};
