// บอร์ดแยกสำหรับควบคุมไฟเลี้ยว MAX7219 4-in-1
// รับคำสั่งจาก Raspberry Pi ผ่าน USB Serial

#include "DisplayConfig.h"
#include "TurnIndicator.h"

char commandBuffer[20];
uint8_t commandIndex = 0;

void processCommand(char command[]) {
  if (strcmp(command, "LEFT") == 0) {
    turnIndicatorSet(TURN_LEFT);
  }
  else if (strcmp(command, "RIGHT") == 0) {
    turnIndicatorSet(TURN_RIGHT);
  }
  else if (strcmp(command, "STOP") == 0 ||
           strcmp(command, "FORWARD") == 0 ||
           strcmp(command, "BACKWARD") == 0) {
    turnIndicatorSet(TURN_OFF);
  }
}

void readRaspberryPiCommand() {
  while (Serial.available() > 0) {
    char received = Serial.read();

    if (received == '\r') {
      continue;
    }

    if (received == '\n') {
      commandBuffer[commandIndex] = '\0';
      processCommand(commandBuffer);
      commandIndex = 0;
    }
    else if (commandIndex < sizeof(commandBuffer) - 1) {
      commandBuffer[commandIndex++] = received;
    }
    else {
      commandIndex = 0;
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  turnIndicatorBegin();
  Serial.println(F("MAX7219:READY"));
}

void loop() {
  readRaspberryPiCommand();
  turnIndicatorUpdate();
}
