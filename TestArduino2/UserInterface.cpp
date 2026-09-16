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

  byte rowPins[ROWS] = {7, 6, 5, 4};
  byte colPins[COLS] = {3, 2, 1, 0};

  Keypad_I2C keypad(
    makeKeymap(keys), rowPins, colPins,
    ROWS, COLS, KEYPAD_ADDRESS
  );

  LiquidCrystal_I2C lcd(LCD_ADDRESS, 16, 2);

  void printJob(const DeliveryJob &job) {
    lcd.print(F("S"));
    lcd.print(job.shelf);
    lcd.print(F(">T"));
    lcd.print(job.table);
  }

  void showReady(const DeliveryJob jobs[], uint8_t jobCount) {
    printJob(jobs[0]);

    if (jobCount == 2) {
      lcd.print(F(" "));
      printJob(jobs[1]);
    }

    lcd.setCursor(0, 1);
    if (jobCount < 2) {
      lcd.print(F("A:Add #=Start"));
    } else {
      lcd.print(F("#=Start *=Reset"));
    }
  }
}

void uiBegin() {
  Wire.begin();
  keypad.begin();

  lcd.init();
  lcd.backlight();
  lcd.clear();
}

char uiReadKey() {
  return keypad.getKey();
}

void uiShowState(
  DeliveryState state,
  uint8_t selectedShelf,
  uint8_t enteredTable,
  const DeliveryJob jobs[],
  uint8_t jobCount,
  uint8_t currentJob
) {
  lcd.clear();

  switch (state) {
    case SELECT_SHELF:
      lcd.print(F("Select shelf"));
      lcd.setCursor(0, 1);
      lcd.print(F("1/2 then B"));
      break;

    case WAIT_FOR_FOOD:
      lcd.print(F("Place food S"));
      lcd.print(selectedShelf);
      lcd.setCursor(0, 1);
      lcd.print(F("Waiting IR..."));
      break;

    case ENTER_TABLE:
      lcd.print(F("Table for S"));
      lcd.print(selectedShelf);
      uiShowTableNumber(enteredTable);
      break;

    case READY_TO_START:
      showReady(jobs, jobCount);
      break;

    case TRAVELLING:
      lcd.print(F("Going T"));
      lcd.print(jobs[currentJob].table);
      lcd.print(F(" S"));
      lcd.print(jobs[currentJob].shelf);
      lcd.setCursor(0, 1);
      lcd.print(F("Please wait..."));
      break;

    case WAIT_FOR_PICKUP:
      lcd.print(F("Arrived T"));
      lcd.print(jobs[currentJob].table);
      lcd.setCursor(0, 1);
      lcd.print(F("Take food S"));
      lcd.print(jobs[currentJob].shelf);
      break;

    case RETURNING_HOME:
      lcd.print(F("All delivered"));
      lcd.setCursor(0, 1);
      lcd.print(F("Returning home"));
      break;
  }
}

void uiShowSelectedShelf(uint8_t shelf) {
  lcd.setCursor(0, 1);
  lcd.print(F("Shelf "));
  lcd.print(shelf);
  lcd.print(F("  B=OK   "));
}

void uiShowTableNumber(uint8_t table) {
  lcd.setCursor(0, 1);
  lcd.print(F("Table:          "));
  lcd.setCursor(7, 1);

  if (table > 0) {
    lcd.print(table);
  } else {
    lcd.print(F("_"));
  }
}

void uiShowShelfAlreadyUsed() {
  lcd.setCursor(0, 1);
  lcd.print(F("Shelf used!     "));
}

void uiShowFoodMissing(uint8_t shelf) {
  lcd.clear();
  lcd.print(F("Food missing!"));
  lcd.setCursor(0, 1);
  lcd.print(F("Check shelf "));
  lcd.print(shelf);
}
