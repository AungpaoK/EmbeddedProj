#include "PiCommunication.h"

#include <string.h>
#include "RobotConfig.h"

namespace {
  char commandBuffer[24];
  uint8_t commandIndex = 0;
  bool discardLongCommand = false;

  PiCommand parseCommand(const char command[]) {
    if (strcmp(command, "LEFT") == 0) return PI_LEFT;
    if (strcmp(command, "RIGHT") == 0) return PI_RIGHT;
    if (strcmp(command, "FORWARD") == 0) return PI_FORWARD;
    if (strcmp(command, "BACKWARD") == 0) return PI_BACKWARD;
    if (strcmp(command, "STOP") == 0) return PI_STOP;
    if (strcmp(command, "ARRIVED") == 0) return PI_ARRIVED;
    if (strcmp(command, "HOME") == 0) return PI_HOME;
    if (strcmp(command, "RESET") == 0) return PI_RESET;
    return PI_UNKNOWN;
  }

  void printStateName(DeliveryState state) {
    switch (state) {
      case SELECT_SHELF:    Serial.println(F("SELECT_SHELF")); break;
      case WAIT_FOR_FOOD:   Serial.println(F("WAIT_FOR_FOOD")); break;
      case ENTER_TABLE:     Serial.println(F("ENTER_TABLE")); break;
      case READY_TO_START:  Serial.println(F("READY_TO_START")); break;
      case TRAVELLING:      Serial.println(F("TRAVELLING")); break;
      case WAIT_FOR_PICKUP: Serial.println(F("WAIT_FOR_PICKUP")); break;
      case RETURNING_HOME:  Serial.println(F("RETURNING_HOME")); break;
    }
  }
}

void communicationBegin() {
  Serial.begin(SERIAL_BAUD_RATE);
}

PiCommand communicationReadCommand() {
  while (Serial.available() > 0) {
    char received = Serial.read();

    if (received == '\r') {
      continue;
    }

    if (received == '\n') {
      if (discardLongCommand) {
        discardLongCommand = false;
        commandIndex = 0;
        return PI_UNKNOWN;
      }

      commandBuffer[commandIndex] = '\0';
      commandIndex = 0;
      return parseCommand(commandBuffer);
    }

    if (!discardLongCommand) {
      if (commandIndex < sizeof(commandBuffer) - 1) {
        commandBuffer[commandIndex++] = received;
      } else {
        discardLongCommand = true;
      }
    }
  }

  return PI_NO_COMMAND;
}

void communicationSendReady() {
  Serial.println(F("TEST_ARDUINO2:READY"));
}

void communicationSendObstacle(bool detected) {
  Serial.print(F("OBSTACLE:"));
  Serial.println(detected ? 1 : 0);
}

void communicationSendFood(uint8_t shelf, bool present) {
  Serial.print(shelf == 1 ? F("FOOD_TOP:") : F("FOOD_BOTTOM:"));
  Serial.println(present ? 1 : 0);
}

void communicationSendFoodMissing(uint8_t shelf) {
  Serial.print(F("FOOD_MISSING:SHELF="));
  Serial.println(shelf);
}

void communicationSendDelivery(const DeliveryJob &job) {
  Serial.print(F("DELIVER:SHELF="));
  Serial.print(job.shelf);
  Serial.print(F(",TABLE="));
  Serial.println(job.table);
}

void communicationSendDelivered(const DeliveryJob &job) {
  Serial.print(F("DELIVERED:SHELF="));
  Serial.print(job.shelf);
  Serial.print(F(",TABLE="));
  Serial.println(job.table);
}

void communicationSendReturnHome() {
  Serial.println(F("RETURN_HOME"));
}

void communicationSendReset() {
  Serial.println(F("DELIVERY:RESET"));
}

void communicationSendStatus(
  float distanceCm,
  bool obstacle,
  bool topFood,
  bool bottomFood,
  DeliveryState state
) {
  Serial.print(F("DISTANCE:"));
  Serial.println(distanceCm, 1);
  communicationSendObstacle(obstacle);
  communicationSendFood(1, topFood);
  communicationSendFood(2, bottomFood);
  Serial.print(F("STATE:"));
  printStateName(state);
}
