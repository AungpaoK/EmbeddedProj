#ifndef ROBOTCONFIG_H
#define ROBOTCONFIG_H

#include <Arduino.h>

// L298 pins

#define IN4 7
#define IN1 8
#define IN2 9
#define ENA 10
#define ENB 11
#define IN3 12

// MAX7219 rear turn-indicator matrix (software SPI)
#define LED_MATRIX_DIN_PIN 4
#define LED_MATRIX_CS_PIN 5
#define LED_MATRIX_CLK_PIN 6

// MAX7219 turn-indicator display settings
constexpr uint8_t MAX7219_COUNT = 4;
constexpr uint8_t MAX7219_BRIGHTNESS = 2;

// Demo mode cycles right five times, then left five times, indefinitely.
// Set false and reflash to select turn signals from motion commands.
constexpr bool TURN_INDICATOR_DEMO_MODE = false;
constexpr uint8_t TURN_INDICATOR_DEMO_CYCLES_PER_SIDE = 5;

constexpr unsigned long TURN_SIGNAL_SEGMENT_MS = 120;
constexpr unsigned long TURN_SIGNAL_HOLD_MS = 300;
constexpr unsigned long TURN_SIGNAL_OFF_MS = 350;
constexpr unsigned long TURN_SIGNAL_CYCLE_MS =
    MAX7219_COUNT * TURN_SIGNAL_SEGMENT_MS + TURN_SIGNAL_HOLD_MS + TURN_SIGNAL_OFF_MS;

// Right Encoder Pins

#define RIGHT_ENC_A A1
#define RIGHT_ENC_B A0

// Left Encoder Pins

#define LEFT_ENC_A A3
#define LEFT_ENC_B A2

// Wheel size

#define WHEEL_RADIUS 0.065 //  6.5 cm
#define WHEEL_BASE 0.343   // 34.3 cm

// Motor Variable
#define TICKS_PER_REV 1920.0

#endif // ROBOTCONFIG_H
