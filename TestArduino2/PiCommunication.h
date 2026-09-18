#ifndef PI_COMMUNICATION_H
#define PI_COMMUNICATION_H

#include <Arduino.h>

void communicationBegin();
void communicationUpdate();

void communicationSendReady();
void communicationSendIr(uint8_t shelf, bool hasFood);
void communicationSendKey(char key);
void communicationSendOverride();
void communicationSendObstacle(bool detected);
void communicationSendDistance(float distanceCm);

#endif
