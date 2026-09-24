#include "PiCommunication.h"

#include <string.h>

#include "RobotConfig.h"
#include "UserInterface.h"

namespace {
  char receiveBuffer[32];
  uint8_t receiveIndex = 0;
  bool discardLongLine = false;

  // รับรูปแบบ LCD:<row>,<text> เช่น LCD:0,Select shelf
  void processLine(char line[]) {
    if (strncmp(line, "LCD:", 4) != 0) {
      return;
    }

    char *comma = strchr(line + 4, ',');
    if (comma == nullptr) {
      return;
    }

    *comma = '\0';
    int row = atoi(line + 4);

    if (row < 0 || row > 1) {
      return;
    }

    uiPrintLine((uint8_t)row, comma + 1);
  }
}

void communicationBegin() {
  Serial.begin(SERIAL_BAUD_RATE);
}

void communicationUpdate() {
  while (Serial.available() > 0) {
    char received = Serial.read();

    if (received == '\r') {
      continue;
    }

    if (received == '\n') {
      if (!discardLongLine) {
        receiveBuffer[receiveIndex] = '\0';
        processLine(receiveBuffer);
      }

      receiveIndex = 0;
      discardLongLine = false;
      continue;
    }

    if (!discardLongLine) {
      if (receiveIndex < sizeof(receiveBuffer) - 1) {
        receiveBuffer[receiveIndex++] = received;
      } else {
        discardLongLine = true;
      }
    }
  }
}

void communicationSendReady() {
  Serial.println(F("STATUS:READY"));
}

void communicationSendIr(uint8_t shelf, bool hasFood) {
  Serial.print(F("IR:"));
  Serial.print(shelf);
  Serial.print(F(","));
  Serial.println(hasFood ? 1 : 0);
}

void communicationSendKey(char key) {
  Serial.print(F("KEY:"));
  Serial.println(key);
}

void communicationSendOverride() {
  Serial.println(F("OVERRIDE"));
}

void communicationSendObstacle(bool detected) {
  Serial.print(F("OBSTACLE:"));
  Serial.println(detected ? 1 : 0);
}

void communicationSendDistance(float distanceCm) {
  Serial.print(F("DISTANCE:"));
  Serial.println(distanceCm, 1);
}
