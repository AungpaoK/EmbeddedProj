  #include "Sensors.h"
#include "RobotConfig.h"

namespace {
  float distanceCm = -1;
  bool obstacle = false;
  bool obstacleChanged = false;

  bool foodPresent[2] = {false, false};
  bool foodRaw[2] = {false, false};
  bool foodChanged[2] = {false, false};
  unsigned long rawChangedTime[2] = {0, 0};

  bool overrideRaw = false;
  bool overrideStable = false;
  bool overridePressed = false;
  unsigned long overrideChangedTime = 0;

  unsigned long lastUltrasonicTime = 0;

  bool readIrPin(uint8_t pin) {
    return digitalRead(pin) == IR_DETECTED_LEVEL;
  }

  float readUltrasonic() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    unsigned long duration = pulseIn(ECHO_PIN, HIGH, 10000);
    if (duration == 0) {
      return -1;
    }

    return duration * 0.0343 / 2.0;
  }

  void updateOneIrSensor(uint8_t index, bool newRaw, unsigned long now) {
    if (newRaw != foodRaw[index]) {
      foodRaw[index] = newRaw;
      rawChangedTime[index] = now;
    }

    if (foodPresent[index] != foodRaw[index] &&
        now - rawChangedTime[index] >= IR_DEBOUNCE_MS) {
      foodPresent[index] = foodRaw[index];
      foodChanged[index] = true;
    }
  }

  void updateIrSensors() {
    unsigned long now = millis();
    updateOneIrSensor(0, readIrPin(IR_TOP_PIN), now);
    updateOneIrSensor(1, readIrPin(IR_BOTTOM_PIN), now);
  }

  void updateOverrideButton() {
    unsigned long now = millis();
    bool newRaw = (digitalRead(MANUAL_OVERRIDE_PIN) == LOW);

    if (newRaw != overrideRaw) {
      overrideRaw = newRaw;
      overrideChangedTime = now;
    }

    if (overrideStable != overrideRaw &&
        now - overrideChangedTime >= BUTTON_DEBOUNCE_MS) {
      overrideStable = overrideRaw;

      if (overrideStable) {
        overridePressed = true;
      }
    }
  }

  void updateUltrasonic() {
    unsigned long now = millis();
    if (now - lastUltrasonicTime < ULTRASONIC_INTERVAL_MS) {
      return;
    }

    lastUltrasonicTime = now;
    distanceCm = readUltrasonic();
    bool oldObstacle = obstacle;

    if (distanceCm < 0) {
      obstacle = false;
    } else if (!obstacle && distanceCm <= STOP_DISTANCE_CM) {
      obstacle = true;
    } else if (obstacle && distanceCm >= CLEAR_DISTANCE_CM) {
      obstacle = false;
    }

    if (obstacle != oldObstacle) {
      obstacleChanged = true;
    }
  }
}

void sensorsBegin() {
  pinMode(IR_TOP_PIN, INPUT_PULLUP);
  pinMode(IR_BOTTOM_PIN, INPUT_PULLUP);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(MANUAL_OVERRIDE_PIN, INPUT_PULLUP);
  digitalWrite(TRIG_PIN, LOW);

  foodPresent[0] = readIrPin(IR_TOP_PIN);
  foodPresent[1] = readIrPin(IR_BOTTOM_PIN);
  foodRaw[0] = foodPresent[0];
  foodRaw[1] = foodPresent[1];
  overrideStable = (digitalRead(MANUAL_OVERRIDE_PIN) == LOW);
  overrideRaw = overrideStable;
}

void sensorsUpdate() {
  updateIrSensors();
  updateUltrasonic();
  updateOverrideButton();
}

float sensorsGetDistanceCm() {
  return distanceCm;
}

bool sensorsHasObstacle() {
  return obstacle;
}

bool sensorsFoodIsPresent(uint8_t shelf) {
  if (shelf < 1 || shelf > 2) {
    return false;
  }
  return foodPresent[shelf - 1];
}

bool sensorsTakeObstacleChanged() {
  bool changed = obstacleChanged;
  obstacleChanged = false;
  return changed;
}

bool sensorsTakeFoodChanged(uint8_t shelf) {
  if (shelf < 1 || shelf > 2) {
    return false;
  }

  uint8_t index = shelf - 1;
  bool changed = foodChanged[index];  
  foodChanged[index] = false;
  return changed;
}

bool sensorsTakeOverridePressed() {
  bool pressed = overridePressed;
  overridePressed = false;
  return pressed;
}
