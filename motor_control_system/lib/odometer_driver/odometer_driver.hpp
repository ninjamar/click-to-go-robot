#pragma once

#include "config.hpp"
#include <Arduino.h>
#include <EnableInterrupt.h>

class OdometerDriver {
public:
    OdometerDriver(uint8_t pinA1, uint8_t pinB1, uint8_t pinA2, uint8_t pinB2)
        : pinA1(pinA1)
        , pinB1(pinB1)
        , pinA2(pinA2)
        , pinB2(pinB2) {
        _instance = this;
    }

    void begin() {
        // give pin interrupt capability
        pinMode(pinA1, INPUT_PULLUP);
        pinMode(pinB1, INPUT_PULLUP);
        pinMode(pinA2, INPUT_PULLUP);
        pinMode(pinB2, INPUT_PULLUP);

        // call isr function when pin changes
        // isr has to be c function. so we don't give a member function which in c is static.
        // but since this static functio needs to access class, we store a pointer to the current class.
        enableInterrupt(pinA1, isrMotor1, CHANGE);
        enableInterrupt(pinB1, isrMotor1, CHANGE);
        enableInterrupt(pinA2, isrMotor2, CHANGE);
        enableInterrupt(pinB2, isrMotor2, CHANGE);

        lastPollTime = millis();
    }

    void poll(float& w1, float& w2) {
        // w is omega. units in angular velocity
        // ISR increments count. Get a copy of it and find time difference

        // Disable interrupts so count doesn't change
        noInterrupts();
        long c1 = count1;
        long c2 = count2;

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
    uint8_t pinA1;
    uint8_t pinA2;
    uint8_t pinB1;
    uint8_t pinB2;

    // volatile since the ISR changes these asyncronously. they can update any time.
    // this prevents the compiler from caching/optimizing it away.
    volatile long count1 = 0;
    volatile long count2 = 0;

    unsigned long lastPollTime = 0;

    static OdometerDriver* _instance; // static so belongs to class not instance

    static float computeW(long count, unsigned long delta) {
        // W = omega = angular velocity
        // w = delta theta / delta t
        // delta theta = (pulses / ppr) * 2pi

        // delta units in milliseconds, need to convert to seconds
        if (delta == 0) {
            // if deltais 0, don't bother
            return 0.0f;
        }
        float sec = delta / 1000.0f;
        return (count / (float)ENCODER_PULSES_PER_REV) * (2 * PI) / sec;
    }

    void handleMotor1() {
        bool a = digitalRead(pinA1);
        bool b = digitalRead(pinB1);
        count1 += (a == b) ? 1 : -1;
    }

    void handleMotor2() {
        bool a = digitalRead(pinA2);
        bool b = digitalRead(pinB2);
        count2 += (a == b) ? 1 : -1;
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
