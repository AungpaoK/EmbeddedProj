#include "TurnIndicator.h"

#include <MD_MAX72xx.h>
#include "DisplayConfig.h"
#include "robotconfig.h"

namespace {
  MD_MAX72XX matrix(
    MD_MAX72XX::FC16_HW,
    LED_MATRIX_DIN_PIN,
    LED_MATRIX_CLK_PIN,
    LED_MATRIX_CS_PIN,
    MAX7219_COUNT
  );

  TurnSignal currentSignal = TURN_OFF;
  bool signalOn = false;
  unsigned long lastPhaseChange = 0;

  uint8_t arrowRight[8] = {
    0b00011000,
    0b00001100,
    0b00000110,
    0b11111111,
    0b11111111,
    0b00000110,
    0b00001100,
    0b00011000
  };

  uint8_t arrowLeft[8] = {
    0b00011000,
    0b00110000,
    0b01100000,
    0b11111111,
    0b11111111,
    0b01100000,
    0b00110000,
    0b00011000
  };

  void drawArrow(uint8_t module, const uint8_t picture[]) {
    uint8_t firstColumn = module * 8;

    for (uint8_t row = 0; row < 8; row++) {
      for (uint8_t column = 0; column < 8; column++) {
        bool ledOn = bitRead(picture[row], 7 - column);
        matrix.setPoint(row, firstColumn + column, ledOn);
      }
    }
  }
}

void turnIndicatorBegin() {
  matrix.begin();
  matrix.control(MD_MAX72XX::INTENSITY, MAX7219_BRIGHTNESS);
  matrix.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
  matrix.clear();
  matrix.update();
}

void turnIndicatorSet(TurnSignal signal) {
  if (currentSignal == signal) {
    return;
  }

  currentSignal = signal;
  signalOn = false;
  lastPhaseChange = millis();
  matrix.clear();
  matrix.update();
}

void turnIndicatorUpdate() {
  if (currentSignal == TURN_OFF) {
    return;
  }

  unsigned long now = millis();
  unsigned long phaseDuration = signalOn ? TURN_SIGNAL_ON_MS : TURN_SIGNAL_OFF_MS;
  if (now - lastPhaseChange < phaseDuration) {
    return;
  }

  lastPhaseChange = now;
  signalOn = !signalOn;
  matrix.clear();
  if (signalOn) {
    for (uint8_t module = 0; module < MAX7219_COUNT; module++) {
      if (currentSignal == TURN_RIGHT) {
        drawArrow(module, arrowRight);
      } else {
        drawArrow(MAX7219_COUNT - 1 - module, arrowLeft);
      }
    }
  }
  matrix.update();
}
