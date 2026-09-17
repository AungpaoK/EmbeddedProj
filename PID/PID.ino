#include "Arduino.h"
#include "robotconfig.h" // for robot & wheel config
#include <MD_MAX72xx.h>

// --- MAX7219 Turn Indicator Pins (Software SPI - using free pins) ---
constexpr uint8_t MAX7219_DIN_PIN = 4;
constexpr uint8_t MAX7219_CLK_PIN = 5;
constexpr uint8_t MAX7219_CS_PIN = 6;
constexpr uint8_t MAX7219_COUNT = 4;
constexpr uint8_t MAX7219_BRIGHTNESS = 2;

enum TurnSignal { TURN_OFF, TURN_LEFT, TURN_RIGHT };

MD_MAX72XX matrix(MD_MAX72XX::FC16_HW, MAX7219_DIN_PIN, MAX7219_CLK_PIN,
                  MAX7219_CS_PIN, MAX7219_COUNT);

TurnSignal currentSignal = TURN_OFF;
uint8_t animationStep = 0;
unsigned long lastAnimationTime = 0;
unsigned long nextAnimationDelay = 130;

const uint8_t arrowRight[8] = {0b00011000, 0b00001100, 0b00000110, 0b11111111,
                               0b11111111, 0b00000110, 0b00001100, 0b00011000};

const uint8_t arrowLeft[8] = {0b00011000, 0b00110000, 0b01100000, 0b11111111,
                              0b11111111, 0b01100000, 0b00110000, 0b00011000};

void drawArrow(uint8_t module, const uint8_t picture[]) {
  uint8_t firstColumn = module * 8;
  for (uint8_t row = 0; row < 8; row++) {
    for (uint8_t column = 0; column < 8; column++) {
      bool ledOn = bitRead(picture[row], 7 - column);
      matrix.setPoint(row, firstColumn + column, ledOn);
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
  nextAnimationDelay = 0;
  lastAnimationTime = millis();
  matrix.clear();
  matrix.update();
}

void turnIndicatorUpdate() {
  if (currentSignal == TURN_OFF) {
    return;
  }

  unsigned long now = millis();
  if (now - lastAnimationTime < nextAnimationDelay) {
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
    nextAnimationDelay = 130;
  } else {
    matrix.clear();
    animationStep = 0;
    nextAnimationDelay = 300;
  }

  matrix.update();
}

// --- Encoder Sign Convention ---
const bool LEFT_ENC_INVERT = false;
const bool RIGHT_ENC_INVERT = true; // Right motor mounted mirrored

volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

// --- Physical Robot Parameters ---
const float METERS_PER_PULSE =
    (2.0 * 3.14159265 * WHEEL_RADIUS) / TICKS_PER_REV;

// --- Target Formula: N = (W / 2) * D * (TPR / (2 * PI * R)) ---
const float TARGET_TURN_TICKS =
    (WHEEL_BASE / 2.0) * PI *
    (TICKS_PER_REV / (2.0 * 3.14159265 * WHEEL_RADIUS));

// --- Timing Control ---
const unsigned long CONTROL_INTERVAL_MS = 20; // 50 Hz Control Loop
const unsigned long DEBUG_PRINT_INTERVAL_MS = 100;

unsigned long lastControlTime = 0;
unsigned long lastDebugPrintMS = 0;
unsigned long stepStartTime = 0;

// Baseline tracking variables
long startLeftTicks = 0;
long startRightTicks = 0;

// --- Speed & Motion Settings ---
float currentRampedSpeedMPS = 0.0;
const float CRUISE_SPEED_MPS = 0.25;
const float TURN_SPEED_MPS = 0.10; // Reduced from 0.15 to limit turn momentum
const float ACCEL_STEP_MPS = 0.005;
const float DECEL_DIST_METERS = 0.40; // Extended from 0.35 for gentler brake
const float TURN_DECEL_FRAC = 0.30;   // Decelerate in last 30% of turn arc

// --- Mission Sequence: Kitchen => Junction => Table 1 => Table 2 => Junction
// => Kitchen ---
enum ActionType {
  ACTION_MOVE,  // Move forward by targetValue (meters)
  ACTION_TURN,  // Turn by targetValue (degrees: +90 = Left, -90 = Right, 180 =
                // U-Turn)
  ACTION_PAUSE, // Pause for targetValue (milliseconds)
  ACTION_DONE   // Stop and complete
};

struct MissionStep {
  ActionType type;
  float targetValue;
  const char *description;
};

// Scenario: Kitchen -> Junction (2m) -> Table 1 (1m) -> Table 2 (2m across
// junction) -> Junction (1m) -> Kitchen (2m)
const MissionStep missionSteps[] = {
    // 1. Kitchen -> Junction (2m)
    {ACTION_MOVE, 2.0, "Kitchen -> Junction (2m)"},
    {ACTION_PAUSE, 1000, "Pause at Junction"},

    // 2. Turn Left 90° toward Table 1, then Move 1m
    {ACTION_TURN, 90.0, "Turn Left 90 deg -> Table 1"},
    {ACTION_PAUSE, 1000, "Pause after Turn"},
    {ACTION_MOVE, 1.0, "Junction -> Table 1 (1m)"},
    {ACTION_PAUSE, 2000, "Serve at Table 1 (Pickup)"},

    // 3. Table 1 -> Table 2 (U-Turn 180°, then 2m straight across Junction)
    {ACTION_TURN, 180.0, "U-Turn 180 deg at Table 1"},
    {ACTION_PAUSE, 1000, "Pause after U-Turn"},
    {ACTION_MOVE, 2.0, "Table 1 -> Table 2 (2m across Junction)"},
    {ACTION_PAUSE, 2000, "Serve at Table 2 (Pickup)"},

    // 4. Table 2 -> Junction (U-Turn 180°, then 1m)
    {ACTION_TURN, 180.0, "U-Turn 180 deg at Table 2"},
    {ACTION_PAUSE, 1000, "Pause after U-Turn"},
    {ACTION_MOVE, 1.0, "Table 2 -> Junction (1m)"},
    {ACTION_PAUSE, 1000, "Pause at Junction"},

    // 5. Turn Left 90° toward Kitchen, then Move 2m
    {ACTION_TURN, 90.0, "Turn Left 90 deg -> Kitchen"},
    {ACTION_PAUSE, 1000, "Pause after Turn"},
    {ACTION_MOVE, 2.0, "Junction -> Kitchen (2m)"},
    {ACTION_PAUSE, 1000, "Arrived at Kitchen"},

    // 6. U-Turn 180° to face front at Kitchen (Ready for next run)
    {ACTION_TURN, 180.0, "U-Turn 180 deg (Face Outward)"},
    {ACTION_DONE, 0.0, "Mission Complete"}};

const uint8_t TOTAL_STEPS = sizeof(missionSteps) / sizeof(missionSteps[0]);
uint8_t currentStepIndex = 0;

float K_sync = 1.5;

// --- PID Controller Struct ---
struct PIDController {
  float Kp = 150.0;
  float Ki = 10.0;
  float Kd = 1.2;

  float errorSum = 0.0;
  float lastActualSpeed = 0.0;
  float actualSpeed = 0.0;
  long prevTicks = 0;

  // Full reset: clears ALL PID state and syncs prevTicks to current position.
  // Call this between phases to prevent stale speed/integral from the
  // previous motion from causing spurious motor commands in the next phase.
  void fullReset(long currentTicks) {
    errorSum = 0.0;
    actualSpeed = 0.0;
    lastActualSpeed = 0.0;
    prevTicks = currentTicks;
  }

  // Legacy alias used by resetBaseline – kept for compatibility.
  void resetIntegral() { errorSum = 0.0; }

  float compute(long currentTicks, float targetSpeedMPS, float dt) {
    long deltaTicks = currentTicks - prevTicks;
    prevTicks = currentTicks;

    float rawActualSpeedMPS = (deltaTicks * METERS_PER_PULSE) / dt;
    actualSpeed = (0.85 * actualSpeed) + (0.15 * rawActualSpeedMPS);

    float speedError = targetSpeedMPS - actualSpeed;
    errorSum += speedError * dt;
    errorSum = constrain(errorSum, -2.0, 2.0);

    float dActualSpeed = (actualSpeed - lastActualSpeed) / dt;
    lastActualSpeed = actualSpeed;

    float feedforwardPWM = (targetSpeedMPS / CRUISE_SPEED_MPS) * 200.0;
    float finalPWM = feedforwardPWM + (Kp * speedError) + (Ki * errorSum) -
                     (Kd * dActualSpeed);

    // Anti-stall floor: use YOUR measured value, not a guess.
    const float MIN_MOVING_PWM = 60.0; // <-- replace with your measured number
    if (abs(targetSpeedMPS) > 0.01 && abs(finalPWM) < MIN_MOVING_PWM) {
      finalPWM = (targetSpeedMPS > 0) ? MIN_MOVING_PWM : -MIN_MOVING_PWM;
    }

    return finalPWM;
  }
};

PIDController pidLeft;
PIDController pidRight;

ISR(PCINT1_vect) {
  static uint8_t lastPortC = 0;
  uint8_t currentPortC = PINC;

  if ((currentPortC & (1 << PC4)) && !(lastPortC & (1 << PC4))) {
    if (currentPortC & (1 << PC5))
      leftEncoderTicks++;
    else
      leftEncoderTicks--;
  }

  if ((currentPortC & (1 << PC0)) && !(lastPortC & (1 << PC0))) {
    if (currentPortC & (1 << PC1))
      rightEncoderTicks++;
    else
      rightEncoderTicks--;
  }
  lastPortC = currentPortC;
}

void setupEncoders() {
  DDRC &= ~0b00110011;
  PORTC |= (1 << PC0) | (1 << PC1) | (1 << PC4) | (1 << PC5);
  PCICR |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT8) | (1 << PCINT12);
}

void readEncoderTicks(long &leftTicksOut, long &rightTicksOut) {
  noInterrupts();
  long rawLeft = leftEncoderTicks;
  long rawRight = rightEncoderTicks;
  interrupts();

  leftTicksOut = LEFT_ENC_INVERT ? -rawLeft : rawLeft;
  rightTicksOut = RIGHT_ENC_INVERT ? -rawRight : rawRight;
}

void driveMotors(int leftPWM, int rightPWM) {
  int pwmL = constrain(abs(leftPWM), 0, 255);
  if (leftPWM > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (leftPWM < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  analogWrite(ENA, pwmL);

  int pwmR = constrain(abs(rightPWM), 0, 255);
  if (rightPWM > 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else if (rightPWM < 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  analogWrite(ENB, pwmR);
}

void resetBaseline(long currentLeft, long currentRight) {
  startLeftTicks = currentLeft;
  startRightTicks = currentRight;
  currentRampedSpeedMPS = 0.0;
  // fullReset syncs prevTicks so the first compute() in the next phase
  // sees deltaTicks == 0 and does not produce a false speed spike.
  pidLeft.fullReset(currentLeft);
  pidRight.fullReset(currentRight);
}

void startStep(uint8_t stepIndex, long curLeft, long curRight,
               unsigned long now) {
  currentStepIndex = stepIndex;
  stepStartTime = now;
  resetBaseline(curLeft, curRight);

  if (currentStepIndex < TOTAL_STEPS) {
    if (missionSteps[currentStepIndex].type == ACTION_TURN) {
      if (missionSteps[currentStepIndex].targetValue >= 0) {
        turnIndicatorSet(TURN_LEFT);
      } else {
        turnIndicatorSet(TURN_RIGHT);
      }
    } else {
      turnIndicatorSet(TURN_OFF);
    }

    Serial.print(F("[STEP "));
    Serial.print(currentStepIndex);
    Serial.print(F("] "));
    Serial.println(missionSteps[currentStepIndex].description);
  }
}

void setup() {
  Serial.begin(115200);

  DDRB |= 0b00111111;
  setupEncoders();
  turnIndicatorBegin();

  pidLeft.Kp = 150.0;
  pidLeft.Ki = 10.0;
  pidLeft.Kd = 1.2;
  pidRight.Kp = 150.0;
  pidRight.Ki = 10.0;
  pidRight.Kd = 1.2;

  startStep(0, 0, 0, millis());
}

void loop() {
  unsigned long now = millis();
  long currentLeftTicks, currentRightTicks;

  if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
    float dt = (now - lastControlTime) / 1000.0;
    lastControlTime = now;

    readEncoderTicks(currentLeftTicks, currentRightTicks);

    if (currentStepIndex < TOTAL_STEPS) {
      switch (missionSteps[currentStepIndex].type) {
      // --- ACTION: MOVE FORWARD ---
      case ACTION_MOVE: {
        float targetDist = missionSteps[currentStepIndex].targetValue;
        long deltaLeft = currentLeftTicks - startLeftTicks;
        long deltaRight = currentRightTicks - startRightTicks;
        float distTraveled =
            ((deltaLeft + deltaRight) / 2.0) * METERS_PER_PULSE;
        float distRemaining = targetDist - distTraveled;

        if (distRemaining <= 0.005) {
          driveMotors(0, 0);
          startStep(currentStepIndex + 1, currentLeftTicks, currentRightTicks,
                    now);
        } else {
          float desiredSpeed = CRUISE_SPEED_MPS;
          if (distRemaining < DECEL_DIST_METERS) {
            desiredSpeed =
                (distRemaining / DECEL_DIST_METERS) * CRUISE_SPEED_MPS;
            if (desiredSpeed < 0.08)
              desiredSpeed = 0.08;
          }

          if (currentRampedSpeedMPS < desiredSpeed) {
            currentRampedSpeedMPS += ACCEL_STEP_MPS;
            if (currentRampedSpeedMPS > desiredSpeed)
              currentRampedSpeedMPS = desiredSpeed;
          } else if (currentRampedSpeedMPS > desiredSpeed) {
            currentRampedSpeedMPS -= ACCEL_STEP_MPS;
            if (currentRampedSpeedMPS < desiredSpeed)
              currentRampedSpeedMPS = desiredSpeed;
          }

          float positionErrorMeters =
              (deltaLeft - deltaRight) * METERS_PER_PULSE;
          float syncCorrectionMPS = positionErrorMeters * K_sync;

          float finalLeftPWM = pidLeft.compute(
              currentLeftTicks, currentRampedSpeedMPS - syncCorrectionMPS, dt);
          float finalRightPWM = pidRight.compute(
              currentRightTicks, currentRampedSpeedMPS + syncCorrectionMPS, dt);

          driveMotors((int)finalLeftPWM, (int)finalRightPWM);
        }
        break;
      }

      // --- ACTION: TURN IN PLACE ---
      case ACTION_TURN: {
        float targetDeg = missionSteps[currentStepIndex].targetValue;
        long targetTicks = (long)((abs(targetDeg) / 180.0) * TARGET_TURN_TICKS);
        long deltaLeft = abs(currentLeftTicks - startLeftTicks);
        long deltaRight = abs(currentRightTicks - startRightTicks);
        long turnTicksProgress = (deltaLeft + deltaRight) / 2;

        if (turnTicksProgress >= targetTicks) {
          driveMotors(0, 0);
          turnIndicatorSet(TURN_OFF);
          startStep(currentStepIndex + 1, currentLeftTicks, currentRightTicks,
                    now);
        } else {
          long turnDecelStart = (long)(targetTicks * (1.0 - TURN_DECEL_FRAC));
          float currentTurnSpeed = TURN_SPEED_MPS;
          if (turnTicksProgress > turnDecelStart) {
            float fraction = (float)(targetTicks - turnTicksProgress) /
                             (targetTicks * TURN_DECEL_FRAC);
            currentTurnSpeed = max(0.04f, TURN_SPEED_MPS * fraction);
          }

          // targetDeg >= 0 (Left Turn): left motor negative, right motor
          // positive targetDeg < 0  (Right Turn): left motor positive, right
          // motor negative
          float dir = (targetDeg >= 0) ? 1.0 : -1.0;

          float finalLeftPWM =
              pidLeft.compute(currentLeftTicks, -dir * currentTurnSpeed, dt);
          float finalRightPWM =
              pidRight.compute(currentRightTicks, dir * currentTurnSpeed, dt);

          driveMotors((int)finalLeftPWM, (int)finalRightPWM);
        }
        break;
      }

      // --- ACTION: PAUSE ---
      case ACTION_PAUSE: {
        driveMotors(0, 0);
        if (now - stepStartTime >=
            (unsigned long)missionSteps[currentStepIndex].targetValue) {
          startStep(currentStepIndex + 1, currentLeftTicks, currentRightTicks,
                    now);
        }
        break;
      }

      // --- ACTION: DONE ---
      case ACTION_DONE: {
        driveMotors(0, 0);
        turnIndicatorSet(TURN_OFF);
        break;
      }
      }
    }
  }

  if (now - lastDebugPrintMS >= DEBUG_PRINT_INTERVAL_MS) {
    lastDebugPrintMS = now;

    readEncoderTicks(currentLeftTicks, currentRightTicks);

    Serial.print(currentLeftTicks);
    Serial.print(",");
    Serial.println(currentRightTicks);
  }

  // Non-blocking animation update for MAX7219 matrix
  turnIndicatorUpdate();
}
