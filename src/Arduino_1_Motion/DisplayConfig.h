#ifndef DISPLAY_CONFIG_H
#define DISPLAY_CONFIG_H

#include <Arduino.h>

constexpr uint8_t MAX7219_COUNT = 4;
constexpr uint8_t MAX7219_BRIGHTNESS = 2;
constexpr unsigned long TURN_SIGNAL_SEGMENT_MS = 120;
constexpr unsigned long TURN_SIGNAL_HOLD_MS = 300;
constexpr unsigned long TURN_SIGNAL_OFF_MS = 350;
constexpr unsigned long SERIAL_BAUD_RATE = 115200;

#endif
