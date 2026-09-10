#ifndef PS2_CONTROLLER_H
#define PS2_CONTROLLER_H

#include <Arduino.h>

#include "robot_config.h"

typedef enum
{
    STOP = 0,

    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    FORWARD_LEFT,
    FORWARD_RIGHT,
    BACKWARD_LEFT,
    BACKWARD_RIGHT,
    Clamp,
    Release

} PS2_Status;

void PS2_ReadData(uint8_t *ps2_data);
PS2_Status PS2_GetStatus(uint8_t *ps2_data);

void apply_motor_from_status(PS2_Status current);
void apply_arm_from_status(PS2_Status current);
void print_debug(PS2_Status status_left, PS2_Status status_right, PS2_Status gripper_status);
#endif