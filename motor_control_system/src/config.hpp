#pragma once
#include "macros.hpp"
#include <Arduino.h>

// Using uint8_t since microcontroller has limited space
const uint8_t QUEUE_SIZE = 4;

// Serial communication
const unsigned long BAUD_RATE = 57600;

// Motor direction pins (L298N IN1/IN2, dual motor)
const int MOTOR_A_IN1_PIN = 6;
const int MOTOR_A_IN2_PIN = 5;

const int MOTOR_B_IN1_PIN = 3;
const int MOTOR_B_IN2_PIN = 4;

// Motor enable/speed pins: L298N ENA/ENB. Use Nano PWM pin. Don't use jumper
// between them
const int MOTOR_A_EN_PIN = 10;
const int MOTOR_B_EN_PIN = 9;

// Odometer / encoder pins for channel
const int ENCODER_MOTORA_CH_A_PIN = 7;
const int ENCODER_MOTORA_CH_B_PIN = 8;

const int ENCODER_MOTORB_CH_A_PIN = 11;
const int ENCODER_MOTORB_CH_B_PIN = 12;

// PPR = blades (12) * 2 edges(hi/lo) / blade * # of active channels.
const uint8_t ENCODER_PULSES_PER_REV_A = 48;
const uint8_t ENCODER_PULSES_PER_REV_B = 24; // Motor B; only CH_A is physically connected

// Geometry — convert encoder pulses into wheel/chassis displacement.
const float WHEEL_RADIUS_M = 0.04;
const float GEAR_RATIO = 64.0f; // Encoder shaft turned ~64 times for 1 wheel rotation
const float TRACK_WIDTH_M = 0.15f; // TODO

// Control loop
const unsigned long CONTROL_TICK_MS = S_TO_MS(0.02f);
const unsigned long MAX_COMMAND_MS = S_TO_MS(120.0f); // emergency stop only
const unsigned long STALL_TIMEOUT_MS = S_TO_MS(3.0f); // abort a command with no odometer progress this long

// PWM
const uint8_t MIN_PWM = 20; // TODO
const uint8_t MAX_PWM = 160; // 256 is PWM ceiling

// PWM ramp: open-loop velocity trapezoid; see main.cpp.
const float MAX_ANGULAR_VEL = 480.0f; // rad/s, motor shaft — TODO: seed only, replace via `calibrate.py velocity-curve`
const float MAX_ANGULAR_ACCEL = 384.0f; // rad/s^2, motor shaft — TODO: hand-tuned seed, refine by feel
const float PWM_PER_RAD_S = (MAX_PWM - MIN_PWM) / MAX_ANGULAR_VEL; // TODO: seed only, replace via `calibrate.py velocity-curve`
