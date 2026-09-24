#include "TurnIndicator.h"

#include <MD_MAX72xx.h>
#include "robotconfig.h"

namespace {
  MD_MAX72XX matrix(
    MD_MAX72XX::FC16_HW,
    LED_MATRIX_DIN_PIN,
    LED_MATRIX_CLK_PIN,
    LED_MATRIX_CS_PIN,
    MAX7219_COUNT
  );

  enum AnimationPhase : uint8_t {
    PHASE_BUILD,
    PHASE_HOLD,
    PHASE_BLANK
  };

  TurnSignal currentSignal = TURN_OFF;
  AnimationPhase animationPhase = PHASE_BUILD;
  uint8_t animationStep = 0;
  unsigned long lastAnimationStep = 0;

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
  animationPhase = PHASE_BUILD;
  animationStep = 0;
  lastAnimationStep = millis();
  matrix.clear();
  matrix.update();
}

void turnIndicatorUpdate() {
  if (currentSignal == TURN_OFF) {
    return;
  }

  unsigned long now = millis();
  unsigned long phaseDuration = TURN_SIGNAL_SEGMENT_MS;
  if (animationPhase == PHASE_HOLD) phaseDuration = TURN_SIGNAL_HOLD_MS;
  if (animationPhase == PHASE_BLANK) phaseDuration = TURN_SIGNAL_OFF_MS;
  if (now - lastAnimationStep < phaseDuration) return;

  lastAnimationStep = now;
  switch (animationPhase) {
    case PHASE_BUILD: {
      uint8_t module = animationStep;
      if (currentSignal == TURN_RIGHT) {
        drawArrow(module, arrowRight);
      } else {
        drawArrow(MAX7219_COUNT - 1 - module, arrowLeft);
      }
      animationStep++;
      if (animationStep >= MAX7219_COUNT) animationPhase = PHASE_HOLD;
      matrix.update();
      break;
    }

    case PHASE_HOLD:
      matrix.clear();
      matrix.update();
      animationPhase = PHASE_BLANK;
      break;

    case PHASE_BLANK:
      animationStep = 0;
      animationPhase = PHASE_BUILD;
      break;
  }
}
