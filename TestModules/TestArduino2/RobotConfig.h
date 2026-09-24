#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <Arduino.h>

// IR Sensor
constexpr uint8_t IR_TOP_PIN = 2;
constexpr uint8_t IR_BOTTOM_PIN = 3;
constexpr uint8_t IR_DETECTED_LEVEL = LOW;
constexpr unsigned long IR_DEBOUNCE_MS = 300;

// Ultrasonic HC-SR04
constexpr uint8_t TRIG_PIN = 6;
constexpr uint8_t ECHO_PIN = 7;
constexpr float STOP_DISTANCE_CM = 50.0;
constexpr float CLEAR_DISTANCE_CM = 60.0;
constexpr unsigned long ULTRASONIC_INTERVAL_MS = 100;

// Manual Override Button (กด = LOW)
constexpr uint8_t MANUAL_OVERRIDE_PIN = 12;
constexpr unsigned long BUTTON_DEBOUNCE_MS = 50;

// I2C
constexpr uint8_t KEYPAD_ADDRESS = 0x20;
constexpr uint8_t LCD_ADDRESS = 0x27;

// Raspberry Pi Serial
constexpr unsigned long SERIAL_BAUD_RATE = 115200;
constexpr unsigned long STATUS_INTERVAL_MS = 1000;

#endif
