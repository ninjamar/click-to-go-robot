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
