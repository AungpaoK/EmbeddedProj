#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <Arduino.h>

// Motor pins
constexpr uint8_t LEFT_MOTOR_PWM_PIN = 10;
constexpr uint8_t LEFT_MOTOR_DIR_PIN = 12;
constexpr uint8_t RIGHT_MOTOR_PWM_PIN = 11;
constexpr uint8_t RIGHT_MOTOR_DIR_PIN = 13;

// PS2 software SPI pins
constexpr uint8_t PS2_DAT_PIN = 14;
constexpr uint8_t PS2_CMD_PIN = 15;
constexpr uint8_t PS2_ATT_PIN = 16;
constexpr uint8_t PS2_CLK_PIN = 17;

// Motion tuning
constexpr uint8_t MOTOR_FULL_SPEED = 200;
constexpr uint8_t MOTOR_TURN_SPEED = 170;
constexpr uint8_t MOTOR_CURVE_SPEED = 140;

#endif