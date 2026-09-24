#ifndef TURN_INDICATOR_H
#define TURN_INDICATOR_H

#include <Arduino.h>

enum TurnSignal : uint8_t {
  TURN_OFF,
  TURN_LEFT,
  TURN_RIGHT
};

void turnIndicatorBegin();
void turnIndicatorSet(TurnSignal signal);
void turnIndicatorUpdate();

#endif
