#include "TurnIndicator.h"

#include <MD_MAX72xx.h>
#include "DisplayConfig.h"

namespace {
  MD_MAX72XX matrix(
    MD_MAX72XX::FC16_HW,
    MAX7219_DATA_PIN,
    MAX7219_CLK_PIN,
    MAX7219_CS_PIN,
    MAX7219_COUNT
  );

  TurnSignal currentSignal = TURN_OFF;
  uint8_t animationStep = 0;
  unsigned long lastAnimationTime = 0;
  unsigned long nextDelay = 130;

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

  void drawArrow(uint8_t module, uint8_t picture[]) {
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
  animationStep = 0;
  nextDelay = 0;
  lastAnimationTime = millis();
  matrix.clear();
  matrix.update();
}

void turnIndicatorUpdate() {
  if (currentSignal == TURN_OFF) {
    return;
  }

  unsigned long now = millis();
  if (now - lastAnimationTime < nextDelay) {
    return;
  }

  lastAnimationTime = now;

  if (animationStep < MAX7219_COUNT) {
    if (currentSignal == TURN_RIGHT) {
      drawArrow(animationStep, arrowRight);
    } else {
      drawArrow(MAX7219_COUNT - 1 - animationStep, arrowLeft);
    }

    animationStep++;
    nextDelay = 130;
  } else {
    matrix.clear();
    animationStep = 0;
    nextDelay = 300;
  }

  matrix.update();
}
