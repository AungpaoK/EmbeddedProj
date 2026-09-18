#include "UserInterface.h"

#include <Wire.h>
#include <Keypad.h>
#include <Keypad_I2C.h>
#include <LiquidCrystal_I2C.h>

#include "RobotConfig.h"

namespace {
  const byte ROWS = 4;
  const byte COLS = 4;

  char keys[ROWS][COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
  };

  // ลำดับนี้ตรงกับบอร์ด PCF8574 ที่ใช้จริง
  byte rowPins[ROWS] = {7, 6, 5, 4};
  byte colPins[COLS] = {3, 2, 1, 0};

  Keypad_I2C keypad(
    makeKeymap(keys), rowPins, colPins,
    ROWS, COLS, KEYPAD_ADDRESS
  );

  LiquidCrystal_I2C lcd(LCD_ADDRESS, 16, 2);
}

void uiBegin() {
  Wire.begin();
  keypad.begin();

  lcd.init();
  lcd.backlight();
  lcd.clear();
  uiPrintLine(0, "Delivery Robot");
  uiPrintLine(1, "Waiting for Pi");
}

char uiReadKey() {
  return keypad.getKey();
}

void uiPrintLine(uint8_t row, const char text[]) {
  if (row > 1) {
    return;
  }

  lcd.setCursor(0, row);
  lcd.print(F("                "));
  lcd.setCursor(0, row);

  for (uint8_t i = 0; i < 16 && text[i] != '\0'; i++) {
    lcd.write(text[i]);
  }
}
