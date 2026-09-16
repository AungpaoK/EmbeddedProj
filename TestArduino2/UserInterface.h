#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

#include <Arduino.h>
#include "RobotTypes.h"

void uiBegin();
char uiReadKey();

void uiShowState(
  DeliveryState state,
  uint8_t selectedShelf,
  uint8_t enteredTable,
  const DeliveryJob jobs[],
  uint8_t jobCount,
  uint8_t currentJob
);

void uiShowSelectedShelf(uint8_t shelf);
void uiShowTableNumber(uint8_t table);
void uiShowShelfAlreadyUsed();
void uiShowFoodMissing(uint8_t shelf);

#endif
