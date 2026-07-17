#pragma once
#include <Arduino.h>

// Using uint8_t since microcontroller has limited space
const uint8_t QUEUE_SIZE = 4;

// Serial communication
const unsigned long BAUD_RATE = 57600;

// Motor pins (L298N dual motor)
const int MOTOR_A_IN1_PIN = 5;
const int MOTOR_A_IN2_PIN = 6;
const int MOTOR_B_IN1_PIN = 3;
const int MOTOR_B_IN2_PIN = 4;

// Odometer / encoder pins (quadrature, motor A = "1", motor B = "2")
const int ENCODER_A1_PIN = 7;
const int ENCODER_B1_PIN = 8;
const int ENCODER_A2_PIN = 9;
const int ENCODER_B2_PIN = 10;

// Encoder characteristics: 12 disc slots x 4 (full quadrature decode) = 48
// pulses per revolution of the motor shaft
const uint8_t ENCODER_PULSES_PER_REV = 48;
