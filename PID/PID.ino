#include "robotconfig.h" // for robot & wheel config
#include "Arduino.h"

// --- Encoder Sign Convention ---
const bool LEFT_ENC_INVERT  = false;
const bool RIGHT_ENC_INVERT = true; // Right motor mounted mirrored

volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

// --- Physical Robot Parameters ---
const float METERS_PER_PULSE = (2.0 * 3.14159265 * WHEEL_RADIUS) / TICKS_PER_REV;

// --- User Turn Equation Formula Variables ---
const float MOVE_DISTANCE_METERS = 2.0;          // Straight target (2 Meters)

// --- Target Formula: N = (W / 2) * D * (TPR / (2 * PI * R)) ---
const float TARGET_TURN_TICKS = (WHEEL_BASE / 2.0) * PI * (TICKS_PER_REV / (2.0 * 3.14159265 * WHEEL_RADIUS));

// --- Timing Control ---
const unsigned long CONTROL_INTERVAL_MS = 20;    // 50 Hz Control Loop
const unsigned long DEBUG_PRINT_INTERVAL_MS = 100;
const unsigned long PAUSE_DURATION_MS = 1000;    // Pause between actions

unsigned long lastControlTime = 0;
unsigned long lastDebugPrintMS = 0;
unsigned long pauseStartTime = 0;

// Baseline tracking variables
long startLeftTicks = 0;
long startRightTicks = 0;

// --- Speed & Motion Settings ---
float currentRampedSpeedMPS = 0.0;
const float CRUISE_SPEED_MPS = 0.25;
const float TURN_SPEED_MPS   = 0.15;
const float ACCEL_STEP_MPS   = 0.005;
const float DECEL_DIST_METERS = 0.20;

// Sequence State: 0 = Move 2m, 1 = Pause, 2 = Turn 180 (Formula N), 3 = Pause, 4 = Move 2m, 5 = Stop
uint8_t runPhase = 0;

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

  void resetIntegral() {
    errorSum = 0.0;
  }

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
    float finalPWM = feedforwardPWM + (Kp * speedError) + (Ki * errorSum) - (Kd * dActualSpeed);

    if (abs(targetSpeedMPS) > 0.01 && abs(finalPWM) < 35) {
      finalPWM = (targetSpeedMPS > 0) ? 35 : -35;
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
    if (currentPortC & (1 << PC5)) leftEncoderTicks--;
    else leftEncoderTicks++;
  }

  if ((currentPortC & (1 << PC0)) && !(lastPortC & (1 << PC0))) {
    if (currentPortC & (1 << PC1)) rightEncoderTicks--;
    else rightEncoderTicks++;
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

  leftTicksOut  = LEFT_ENC_INVERT  ? -rawLeft  : rawLeft;
  rightTicksOut = RIGHT_ENC_INVERT ? -rawRight : rawRight;
}

void driveMotors(int leftPWM, int rightPWM) {
  int pwmL = constrain(abs(leftPWM), 0, 255);
  if (leftPWM > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else if (leftPWM < 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  }
  analogWrite(ENA, pwmL);

  int pwmR = constrain(abs(rightPWM), 0, 255);
  if (rightPWM > 0) {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  } else if (rightPWM < 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  }
  analogWrite(ENB, pwmR);
}

void resetBaseline(long currentLeft, long currentRight) {
  startLeftTicks = currentLeft;
  startRightTicks = currentRight;
  currentRampedSpeedMPS = 0.0;
  pidLeft.resetIntegral();
  pidRight.resetIntegral();
}

void setup() {
  Serial.begin(115200);

  DDRB |= 0b00111111;
  setupEncoders();

  pidLeft.Kp = 150.0; pidLeft.Ki = 10.0; pidLeft.Kd = 1.2;
  pidRight.Kp = 150.0; pidRight.Ki = 10.0; pidRight.Kd = 1.2;
}

void loop() {
  unsigned long now = millis();
  long currentLeftTicks, currentRightTicks;

  if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
    float dt = (now - lastControlTime) / 1000.0;
    lastControlTime = now;

    readEncoderTicks(currentLeftTicks, currentRightTicks);

    switch (runPhase) {
      // --- PHASE 0 & 4: FORWARD DISTANCE (2.0 METERS) ---
      case 0:
      case 4: {
        long deltaLeft  = currentLeftTicks - startLeftTicks;
        long deltaRight = currentRightTicks - startRightTicks;
        float distTraveled = ((deltaLeft + deltaRight) / 2.0) * METERS_PER_PULSE;
        float distRemaining = MOVE_DISTANCE_METERS - distTraveled;

        if (distRemaining <= 0.005) {
          driveMotors(0, 0);
          pauseStartTime = now;
          runPhase++;
        } else {
          float desiredSpeed = CRUISE_SPEED_MPS;
          if (distRemaining < DECEL_DIST_METERS) {
            desiredSpeed = (distRemaining / DECEL_DIST_METERS) * CRUISE_SPEED_MPS;
            if (desiredSpeed < 0.04) desiredSpeed = 0.04;
          }

          if (currentRampedSpeedMPS < desiredSpeed) {
            currentRampedSpeedMPS += ACCEL_STEP_MPS;
            if (currentRampedSpeedMPS > desiredSpeed) currentRampedSpeedMPS = desiredSpeed;
          } else if (currentRampedSpeedMPS > desiredSpeed) {
            currentRampedSpeedMPS -= ACCEL_STEP_MPS;
            if (currentRampedSpeedMPS < desiredSpeed) currentRampedSpeedMPS = desiredSpeed;
          }

          float positionErrorMeters = (deltaLeft - deltaRight) * METERS_PER_PULSE;
          float syncCorrectionMPS = positionErrorMeters * K_sync;

          float finalLeftPWM  = pidLeft.compute(currentLeftTicks, currentRampedSpeedMPS - syncCorrectionMPS, dt);
          float finalRightPWM = pidRight.compute(currentRightTicks, currentRampedSpeedMPS + syncCorrectionMPS, dt);

          driveMotors((int)finalLeftPWM, (int)finalRightPWM);
        }
        break;
      }

      // --- PHASE 1 & 3: PAUSES ---
      case 1:
      case 3: {
        driveMotors(0, 0);
        if (now - pauseStartTime >= PAUSE_DURATION_MS) {
          resetBaseline(currentLeftTicks, currentRightTicks);
          runPhase++;
        }
        break;
      }

      // --- PHASE 2: 180° TURN USING YOUR N FORMULA ---
      case 2: {
        long deltaLeft  = abs(currentLeftTicks - startLeftTicks);
        long deltaRight = abs(currentRightTicks - startRightTicks);
        long turnTicksProgress = (deltaLeft + deltaRight) / 2;

        if (turnTicksProgress >= (long)TARGET_TURN_TICKS) {
          driveMotors(0, 0);
          pauseStartTime = now;
          runPhase = 3;
        } else {
          float finalLeftPWM  = pidLeft.compute(currentLeftTicks, -TURN_SPEED_MPS, dt);
          float finalRightPWM = pidRight.compute(currentRightTicks, TURN_SPEED_MPS, dt);

          driveMotors((int)finalLeftPWM, (int)finalRightPWM);
        }
        break;
      }

      // --- PHASE 5: COMPLETE ---
      case 5: {
        driveMotors(0, 0);
        break;
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
}
