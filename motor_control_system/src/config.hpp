#pragma once
#include <Arduino.h>

// Using uint8_t since microcontroller has limited space
const uint8_t QUEUE_SIZE = 4;

// Serial communication
const unsigned long BAUD_RATE = 57600;

// Motor pins (L298N)
const int MOTOR_IN1_PIN = 5;
const int MOTOR_IN2_PIN = 6;
