#pragma once
#include <L298N.h>

class MotorDriver {
public:
    MotorDriver(uint8_t pinEnable, uint8_t pinIN1, uint8_t pinIN2)
        : _motor(pinEnable, pinIN1, pinIN2)
    {
    } // initialzer list
    MotorDriver(uint8_t pinIN1, uint8_t pinIN2)
        : _motor(pinIN1, pinIN2)
    {
    }

    void move(uint8_t duration)
    {
        _motor.forwardFor(duration);
    }

    void rotate(uint8_t arg)
    {
    }

    void stop()
    {
        _motor.stop();
    }

    L298N& getMotor()
    {
        return _motor;
    }

private:
    L298N _motor;
};
