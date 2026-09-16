#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <Arduino.h>

enum DeliveryState : uint8_t {
  SELECT_SHELF,
  WAIT_FOR_FOOD,
  ENTER_TABLE,
  READY_TO_START,
  TRAVELLING,
  WAIT_FOR_PICKUP,
  RETURNING_HOME
};

enum TurnSignal : uint8_t {
  TURN_OFF,
  TURN_LEFT,
  TURN_RIGHT
};

enum PiCommand : uint8_t {
  PI_NO_COMMAND,
  PI_LEFT,
  PI_RIGHT,
  PI_FORWARD,
  PI_BACKWARD,
  PI_STOP,
  PI_ARRIVED,
  PI_HOME,
  PI_RESET,
  PI_UNKNOWN
};

struct DeliveryJob {
  uint8_t shelf;
  uint8_t table;
};

#endif
