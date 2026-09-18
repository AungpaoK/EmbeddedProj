// Sketch ทดสอบ Keypad ผ่าน PCF8574 ด้วย Hardware I2C
// PCF8574 SDA -> A4, SCL -> A5, VCC -> 5V, GND -> GND

#include "Arduino1Keypad.h"

void setup() {
  Serial.begin(115200);
  Arduino1Keypad::begin();
  Serial.println(F("ARDUINO1_KEYPAD:READY"));
}

void loop() {
  Arduino1Keypad::updateAndSendToPi(Serial);
}
