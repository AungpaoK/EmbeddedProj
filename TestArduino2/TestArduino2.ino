/*
  Arduino Uno #2 - Shelf & User Interface Controller

  หน้าที่ของบอร์ดนี้:
    - อ่าน IR Sensor ชั้น 1 และ 2
    - อ่าน Ultrasonic ตรวจสิ่งกีดขวาง
    - อ่าน Keypad 4x4 ผ่าน PCF8574
    - อ่านปุ่ม Manual Override
    - แสดงข้อความ LCD ที่ได้รับจาก Raspberry Pi

  Main Delivery FSM อยู่บน Raspberry Pi ไม่ได้อยู่ในบอร์ดนี้
*/

#include "RobotConfig.h"
#include "Sensors.h"
#include "UserInterface.h"
#include "PiCommunication.h"

unsigned long lastStatusTime = 0;

void sendChangedInputs() {
  if (sensorsTakeObstacleChanged()) {
    communicationSendObstacle(sensorsHasObstacle());
  }

  for (uint8_t shelf = 1; shelf <= 2; shelf++) {
    if (sensorsTakeFoodChanged(shelf)) {
      communicationSendIr(shelf, sensorsFoodIsPresent(shelf));
    }
  }

  if (sensorsTakeOverridePressed()) {
    communicationSendOverride();
  }

  char key = uiReadKey();
  if (key) {
    communicationSendKey(key);
  }
}

void sendPeriodicStatus() {
  unsigned long now = millis();
  if (now - lastStatusTime < STATUS_INTERVAL_MS) {
    return;
  }

  lastStatusTime = now;
  communicationSendIr(1, sensorsFoodIsPresent(1));
  communicationSendIr(2, sensorsFoodIsPresent(2));
  communicationSendDistance(sensorsGetDistanceCm());
  communicationSendObstacle(sensorsHasObstacle());
}

void setup() {
  communicationBegin();
  sensorsBegin();
  uiBegin();

  communicationSendReady();
  communicationSendIr(1, sensorsFoodIsPresent(1));
  communicationSendIr(2, sensorsFoodIsPresent(2));
}

void loop() {
  communicationUpdate();
  sensorsUpdate();
  sendChangedInputs();
  sendPeriodicStatus();
}
