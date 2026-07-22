#pragma once
#include "macros.hpp"
#include <Arduino.h>

// Using uint8_t since microcontroller has limited space
const uint8_t QUEUE_SIZE = 4;

// Serial communication
const unsigned long BAUD_RATE = 57600;

// Motor direction pins (L298N IN1/IN2, dual motor)
const int MOTOR_A_IN1_PIN = 5;
const int MOTOR_A_IN2_PIN = 6;
const int MOTOR_B_IN1_PIN = 3;
const int MOTOR_B_IN2_PIN = 4;

// Motor enable/speed pins (L298N ENA/ENB) — must be PWM-capable (3,5,6,9,10,11
// on a Nano). These must actually be wired to the Nano, not jumpered straight
// to VCC on the L298N board — if EN is tied high in hardware, setSpeed() has
// no effect no matter what value firmware sends.
const int MOTOR_A_EN_PIN = 11;
const int MOTOR_B_EN_PIN = 9;

// Odometer / encoder pins (quadrature, CH_A/CH_B are the two phase channels of each encoder)
// NOTE: motor B's CH_A moved from pin 9 to pin 12 to free pin 9 (PWM-capable)
// for MOTOR_B_EN_PIN above — encoder channels don't need PWM, EN does.
const int ENCODER_MOTORA_CH_A_PIN = 7;
const int ENCODER_MOTORA_CH_B_PIN = 8;
const int ENCODER_MOTORB_CH_A_PIN = 10;
const int ENCODER_MOTORB_CH_B_PIN = 12;

// Encoder characteristics: pulses per revolution of the motor shaft, counting
// raw CHANGE edges from both channels (see odometer_driver.hpp - the two
// readers aren't a manufactured quadrature pair, so this is no longer a
// clean "slots x 4" figure). PLACEHOLDER: recalibrate empirically by counting
// pulses over a known number of full turns.
const uint8_t ENCODER_PULSES_PER_REV = 48;

// Geometry — convert encoder pulses into wheel/chassis displacement.
// PLACEHOLDERS: measure with a ruler / count gearbox teeth before relying on
// MOVE_DIST or ROTATE_ANGLE accuracy.
const float WHEEL_RADIUS_M = 0.04f;
const float GEAR_RATIO = 1.0f;
const float TRACK_WIDTH_M = 0.15f;

// Control loop
const unsigned long CONTROL_TICK_MS = S_TO_MS(0.02f);
const unsigned long MAX_COMMAND_MS = S_TO_MS(15.0f); // safety cutoff for any long-running active command

// PWM ramp: fixed ramp-up duration, fixed ramp-down margin — not a
// proportional curve. See computeRampedPwm() in main.cpp.
const unsigned long RAMP_UP_MS = S_TO_MS(0.3f);
const unsigned long RAMP_DOWN_MS = S_TO_MS(0.3f); // MOVE / ROTATE (time-based)
const float RAMP_DOWN_DISTANCE_M = 0.02f; // MOVE_DIST
const float RAMP_DOWN_ANGLE_RAD = DEG_TO_RADIAN(5.15662); // ROTATE_ANGLE

// PLACEHOLDER: MIN_PWM must be found empirically (lowest duty cycle that
// reliably overcomes stiction and starts the motor turning under load).
const uint8_t MIN_PWM = 60;
const uint8_t MAX_PWM = 255;
