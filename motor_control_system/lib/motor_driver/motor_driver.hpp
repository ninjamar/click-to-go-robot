#pragma once
#include <L298N.h>
#include <L298NX2.h>

class MotorDriver {
public:
    MotorDriver(uint8_t pinEnA, uint8_t pinIN1_A, uint8_t pinIN2_A,
        uint8_t pinEnB, uint8_t pinIN1_B, uint8_t pinIN2_B)
        : _motors(pinEnA, pinIN1_A, pinIN2_A, pinEnB, pinIN1_B, pinIN2_B) {
    }

    // Sets both motors to the same direction at the given duty cycle.
    // No internal timing — caller (main.cpp's control tick) is the sole
    // authority on when to stop; this just applies "right now".
    void driveMove(int8_t direction, uint8_t pwm) {
        applyDirection(direction, /*opposite=*/false);
        _motors.setSpeed(pwm);
    }

    void driveRotate(int8_t direction, uint8_t pwm) {
        // In-place rotation
        applyDirection(direction, /*opposite=*/true);
        _motors.setSpeed(pwm);
    }

    void stop() {
        _motors.stop();
    }

private:
    L298NX2 _motors;

    void applyDirection(int8_t direction, bool opposite) {
        if (direction > 0) {
            _motors.forwardA();
            opposite ? _motors.forwardB() : _motors.backwardB();
        } else if (direction < 0) {
            _motors.backwardA();
            opposite ? _motors.backwardB() : _motors.forwardB();
        } else {
            _motors.stop();
        }
    }
};
