#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

void sensorsBegin();
void sensorsUpdate();

float sensorsGetDistanceCm();
bool sensorsHasObstacle();
bool sensorsFoodIsPresent(uint8_t shelf);

// คืนค่า true เพียงครั้งเดียวเมื่อสถานะเปลี่ยน
bool sensorsTakeObstacleChanged();
bool sensorsTakeFoodChanged(uint8_t shelf);

#endif
