#ifndef ARDUINO1_KEYPAD_H
#define ARDUINO1_KEYPAD_H

/*
  Keypad 4x4 ผ่าน PCF8574 สำหรับ Arduino #1
  ==========================================

  ใช้ Hardware I2C ของ Arduino Uno ผ่านไลบรารี Wire และ Keypad_I2C

  การต่อสาย:
    PCF8574 SDA -> Arduino A4 (SDA)
    PCF8574 SCL -> Arduino A5 (SCL)
    PCF8574 VCC -> Arduino 5V
    PCF8574 GND -> Arduino GND

  ขาปัจจุบัน:
    A4/A5 ใช้เป็น SDA/SCL สำหรับ PCF8574
    Encoder ขวาใช้ A0/A1 และ Encoder ซ้ายใช้ A2/A3 จึงไม่ชนกับ I2C

  วิธีเรียกจาก Arduino_1_Motion.ino:

    #include "Arduino1Keypad.h"

    void setup() {
      Serial.begin(115200);
      Arduino1Keypad::begin();
      // setup ระบบ Motion อื่น ๆ
    }

    void loop() {
      // เรียกทุก loop ฟังก์ชันจะส่ง KEY:<ปุ่ม> ให้ Raspberry Pi
      Arduino1Keypad::updateAndSendToPi(Serial);

      // เรียก Motion/PID loop ตามเดิม
    }

  หมายเหตุ:
    - Address เริ่มต้นของ PCF8574 คือ 0x20
    - ถ้าโมดูลใช้ address อื่น ให้แก้ PCF8574_ADDRESS
    - Raspberry Pi ต้องอ่าน KEY: จาก Serial ของ Arduino #1
*/

#include <Arduino.h>
#include <Wire.h>
#include <Keypad.h>
#include <Keypad_I2C.h>

namespace Arduino1Keypad {

constexpr byte ROWS = 4;
constexpr byte COLS = 4;
constexpr byte PCF8574_ADDRESS = 0x20;

// ลำดับขาตรงกับ PCF8574 adapter ที่ใช้ในโปรเจกต์
byte rowPins[ROWS] = {7, 6, 5, 4};
byte colPins[COLS] = {3, 2, 1, 0};

char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

Keypad_I2C keypad(
  makeKeymap(keys),
  rowPins,
  colPins,
  ROWS,
  COLS,
  PCF8574_ADDRESS
);

inline void begin() {
  Wire.begin();
  keypad.begin();
  keypad.setDebounceTime(40);
}

// คืนตัวอักษรหนึ่งครั้งเมื่อมีการกดปุ่มใหม่ ถ้าไม่มีปุ่มจะคืน NO_KEY
inline char readKey() {
  return keypad.getKey();
}

// อ่านปุ่มแล้วส่ง KEY:<ปุ่ม> ให้ Raspberry Pi
inline void updateAndSendToPi(Stream &serialPort) {
  static unsigned long lastScanMs = 0;
  const unsigned long nowMs = millis();
  if (nowMs - lastScanMs < 10) {
    return;
  }
  lastScanMs = nowMs;

  char key = readKey();

  if (key != NO_KEY) {
    serialPort.print(F("KEY:"));
    serialPort.println(key);
  }
}

}  // namespace Arduino1Keypad

#endif
