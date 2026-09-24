#ifndef DISPLAY_CONFIG_H
#define DISPLAY_CONFIG_H

#include <Arduino.h>

// Software SPI pins from docs/design.md (DIN, CS, CLK).
constexpr uint8_t MAX7219_DATA_PIN = 2;
constexpr uint8_t MAX7219_CS_PIN = 3;
constexpr uint8_t MAX7219_CLK_PIN = 4;
constexpr uint8_t MAX7219_COUNT = 4;
constexpr uint8_t MAX7219_BRIGHTNESS = 2;
constexpr unsigned long SERIAL_BAUD_RATE = 115200;

#endif
