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
        // give pin interrupt capability
        pinMode(pin1ChA, INPUT_PULLUP);
        pinMode(pin1ChB, INPUT_PULLUP);
        pinMode(pin2ChA, INPUT_PULLUP);
        pinMode(pin2ChB, INPUT_PULLUP);

        // call isr function when pin changes
        // isr has to be c function. so we don't give a member function which in c is static.
        // but since this static functio needs to access class, we store a pointer to the current class.

        // TODO: MOTOR_A
        // enableInterrupt(pin1ChA, isrMotor1, CHANGE);
        // enableInterrupt(pin1ChB, isrMotor1, CHANGE);

        // TODO: enable/disable for testing
        enableInterrupt(pin2ChA, isrMotor2, CHANGE);
        enableInterrupt(pin2ChB, isrMotor2, CHANGE);

        lastPollTime = millis();
    }

    void poll(float& w1, float& w2) {
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

        w1 = computeW(c1, delta);
        w2 = computeW(c2, delta);
    }

private:
    // interrupt pins
    uint8_t pin1ChA;
    uint8_t pin1ChB;
    uint8_t pin2ChA;
    uint8_t pin2ChB;

    // volatile since the ISR changes these asyncronously. they can update any time.
    // this prevents the compiler from caching/optimizing it away.
    // Unsigned pulse counts (magnitude only) - see handleMotor1/handleMotor2.
    volatile unsigned long count1 = 0;
    volatile unsigned long count2 = 0;

    unsigned long lastPollTime = 0;

    static OdometerDriver* _instance; // static so belongs to class not instance

    static float computeW(unsigned long count, unsigned long delta) {
        // W = omega = angular velocity
        // w = delta theta / delta t
        // delta theta = (pulses / ppr) * 2pi

        // delta units in milliseconds, need to convert to seconds
        if (delta == 0) {
            // if deltais 0, don't bother
            return 0.0f;
        }
        float sec = MS_TO_S(delta);
        return (count / static_cast<float>(ENCODER_PULSES_PER_REV)) * (2 * PI) / sec;
    }

    // The two readers are two independently-mounted photointerrupters, not a
    // manufactured quadrature pair - there's no guarantee they sit at a clean
    // 90 degree phase offset, so a Gray-code/quadrature decode can't reliably
    // tell direction from the two channels (near-simultaneous A/B edges make
    // it a timing race). Instead we just tally raw edges from both channels
    // as a magnitude-only pulse count; direction is supplied by the caller
    // from the commanded motor direction instead of being derived here.
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
