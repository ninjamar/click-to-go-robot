#pragma once

#include "config.hpp"
#include "macros.hpp"
#include <Arduino.h>
#include <EnableInterrupt.h>

class OdometerDriver {
public:
    OdometerDriver(uint8_t pin1ChA, uint8_t pin1ChB, uint8_t pin2ChA, uint8_t pin2ChB)
        : pin1ChA(pin1ChA)
        , pin1ChB(pin1ChB)
        , pin2ChA(pin2ChA)
        , pin2ChB(pin2ChB) {
        _instance = this;
    }

    void begin() {
        pinMode(pin1ChA, INPUT_PULLUP);
        pinMode(pin1ChB, INPUT_PULLUP);
        pinMode(pin2ChA, INPUT_PULLUP);
        pinMode(pin2ChB, INPUT_PULLUP);

        // ISRs must be free C functions, so _instance lets them reach this object.
        // TODO: Motor A
        // enableInterrupt(pin1ChA, isrMotor1, CHANGE);
        // enableInterrupt(pin1ChB, isrMotor1, CHANGE);

        // Motor B only has 1 channel
        enableInterrupt(pin2ChA, isrMotor2, CHANGE);
        // enableInterrupt(pin2ChB, isrMotor2, CHANGE);

        lastPollTime = millis();
    }

    // dtSec is the real elapsed time covered, so callers can recover exact angle as w * dtSec.
    void poll(float& w1, float& w2, float& dtSec) {
        // w is omega. units in angular velocity
        // ISR increments count. Get a copy of it and find time difference

        // Disable interrupts so count doesn't change
        noInterrupts();
        unsigned long c1 = count1;
        unsigned long c2 = count2;

        unsigned long now = millis();
        unsigned long delta = now - lastPollTime;

        count1 = 0;
        count2 = 0;
        lastPollTime = now;

        interrupts();

        dtSec = MS_TO_S(delta);
        w1 = computeW(c1, delta, ENCODER_PULSES_PER_REV_A);
        w2 = computeW(c2, delta, ENCODER_PULSES_PER_REV_B);
    }

private:
    // interrupt pins
    uint8_t pin1ChA;
    uint8_t pin1ChB;
    uint8_t pin2ChA;
    uint8_t pin2ChB;

    // volatile: the ISR updates these asynchronously, any time.
    volatile unsigned long count1 = 0;
    volatile unsigned long count2 = 0;

    unsigned long lastPollTime = 0;

    // static means method/field is tied to class not instance
    static OdometerDriver* _instance; // lets the free-function ISRs reach this object

    static float computeW(unsigned long count, unsigned long delta, uint8_t ppr) {
        // w = (pulses / ppr) * 2pi / dt
        if (delta == 0) {
            return 0.0f; // avoid divide-by-zero below
        }
        float sec = MS_TO_S(delta);
        return (count / static_cast<float>(ppr)) * (2 * PI) / sec;
    }

    void handleMotor1() {
        count1++;
    }

    void handleMotor2() {
        count2++;
    }

    static void isrMotor1() {
        _instance->handleMotor1();
    }

    static void isrMotor2() {
        _instance->handleMotor2();
    }
};

// Doesn't belong to any specific instance so we set it here.
OdometerDriver* OdometerDriver::_instance = nullptr;
