// Arduino Uno #2 - โปรแกรมหลัก
// รายละเอียดของแต่ละระบบแยกอยู่ในไฟล์ .h และ .cpp

#include "RobotConfig.h"
#include "RobotTypes.h"
#include "Sensors.h"
#include "UserInterface.h"
#include "TurnIndicator.h"
#include "PiCommunication.h"
#include "DeliveryStateMachine.h"

unsigned long lastStatusTime = 0;

void handlePiCommand(PiCommand command) {
  // คำสั่งทิศทางใช้ควบคุมไฟลูกศร
  if (command == PI_LEFT) {
    turnIndicatorSet(TURN_LEFT);
  }
  else if (command == PI_RIGHT) {
    turnIndicatorSet(TURN_RIGHT);
  }
  else if (command == PI_FORWARD ||
           command == PI_BACKWARD ||
           command == PI_STOP) {
    turnIndicatorSet(TURN_OFF);
  }

  // ARRIVED, HOME และ RESET ส่งให้ State Machine จัดการ
  deliveryHandlePiCommand(command);
}

void sendChangedSensorValues() {
  if (sensorsTakeObstacleChanged()) {
    communicationSendObstacle(sensorsHasObstacle());
  }

  for (uint8_t shelf = 1; shelf <= 2; shelf++) {
    if (sensorsTakeFoodChanged(shelf)) {
      bool foodPresent = sensorsFoodIsPresent(shelf);
      communicationSendFood(shelf, foodPresent);
      deliveryHandleFoodChange(shelf, foodPresent);
    }
  }
}

void sendPeriodicStatus() {
  unsigned long now = millis();
  if (now - lastStatusTime < STATUS_INTERVAL_MS) {
    return;
  }

  lastStatusTime = now;
  communicationSendStatus(
    sensorsGetDistanceCm(),
    sensorsHasObstacle(),
    sensorsFoodIsPresent(1),
    sensorsFoodIsPresent(2),
    deliveryGetState()
  );
}

void setup() {
  communicationBegin();
  sensorsBegin();
  uiBegin();
  turnIndicatorBegin();
  deliveryBegin();

  communicationSendReady();
}

void loop() {
  PiCommand command = communicationReadCommand();
  if (command != PI_NO_COMMAND && command != PI_UNKNOWN) {
    handlePiCommand(command);
  }

  sensorsUpdate();
  sendChangedSensorValues();

  deliveryUpdate();
  turnIndicatorUpdate();
  sendPeriodicStatus();
}
