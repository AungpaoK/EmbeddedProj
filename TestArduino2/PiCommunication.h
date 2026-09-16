#ifndef PI_COMMUNICATION_H
#define PI_COMMUNICATION_H

#include <Arduino.h>
#include "RobotTypes.h"

void communicationBegin();
PiCommand communicationReadCommand();

void communicationSendReady();
void communicationSendObstacle(bool detected);
void communicationSendFood(uint8_t shelf, bool present);
void communicationSendFoodMissing(uint8_t shelf);
void communicationSendDelivery(const DeliveryJob &job);
void communicationSendDelivered(const DeliveryJob &job);
void communicationSendReturnHome();
void communicationSendReset();
void communicationSendStatus(
  float distanceCm,
  bool obstacle,
  bool topFood,
  bool bottomFood,
  DeliveryState state
);

#endif
