#ifndef DISPLAY_CONFIG_H
#define DISPLAY_CONFIG_H

#include <Arduino.h>

constexpr uint8_t MAX7219_COUNT = 4;
constexpr uint8_t MAX7219_BRIGHTNESS = 2;
// Demo until the chassis is ready: 5 right cycles, then 5 left cycles, forever.
// Set false and reflash to use motion-command-driven indicators.
constexpr bool TURN_INDICATOR_DEMO_MODE = true;
constexpr uint8_t TURN_INDICATOR_DEMO_CYCLES_PER_SIDE = 5;
constexpr unsigned long TURN_SIGNAL_SEGMENT_MS = 120;
constexpr unsigned long TURN_SIGNAL_HOLD_MS = 300;
constexpr unsigned long TURN_SIGNAL_OFF_MS = 350;
constexpr unsigned long TURN_SIGNAL_CYCLE_MS =
    MAX7219_COUNT * TURN_SIGNAL_SEGMENT_MS + TURN_SIGNAL_HOLD_MS + TURN_SIGNAL_OFF_MS;
constexpr unsigned long SERIAL_BAUD_RATE = 115200;

#endif
