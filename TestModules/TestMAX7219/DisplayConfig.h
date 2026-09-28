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
// Keep the test display dim so a phone camera can capture individual LEDs
// without clipping them into a solid block.
constexpr uint8_t MAX7219_BRIGHTNESS = 0;

// Set true only for standalone indicator debugging. Keep false for robot use,
// where the Raspberry Pi selects the requested turn signal.
constexpr bool TURN_INDICATOR_DEMO_MODE = false;
constexpr uint8_t TURN_INDICATOR_DEMO_CYCLES_PER_SIDE = 5;

// In normal mode the Raspberry Pi selects left, right, or off. Debug demo mode
// above ignores Pi indicator commands and alternates sides autonomously.
constexpr unsigned long TURN_SIGNAL_SEGMENT_MS = 350;
constexpr unsigned long TURN_SIGNAL_HOLD_MS = 1200;
constexpr unsigned long TURN_SIGNAL_OFF_MS = 900;

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
