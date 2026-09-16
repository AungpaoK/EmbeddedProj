#ifndef DELIVERY_STATE_MACHINE_H
#define DELIVERY_STATE_MACHINE_H

#include <Arduino.h>
#include "RobotTypes.h"

void deliveryBegin();
void deliveryUpdate();
void deliveryHandleFoodChange(uint8_t shelf, bool foodPresent);
void deliveryHandlePiCommand(PiCommand command);
DeliveryState deliveryGetState();

#endif
