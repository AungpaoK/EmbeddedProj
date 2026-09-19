#include "robotconfig.h"
#include <Arduino.h>

volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

// --- Timing Control ---
const unsigned long CONTROL_INTERVAL_MS = 20;       // 50 Hz Control Loop
const unsigned long DEBUG_PRINT_INTERVAL_MS = 100;
const unsigned long PAUSE_BETWEEN_STEPS_MS = 500;    // Pause between maneuvers

unsigned long lastControlTime = 0;
unsigned long lastDebugPrintMS = 0;
unsigned long stepPauseStartTime = 0;

// --- Physical Robot Parameters ---
const float METERS_PER_TICK = (2.0 * PI * WHEEL_RADIUS) / TICKS_PER_REV;
const float WHEEL_BASE_M = 0.343;                  // 34.3 cm Wheelbase

// --- Motion States Enum ---
enum State {
  ACCELERATING,
  CRUISING,
  DECELERATING,
  PAUSING,
  COMPLETED
};
State currentState = ACCELERATING;

enum MotionType { DRIVE_STRAIGHT, TANK_TURN };

// --- Motion Control Variables ---
MotionType currentMotionType = DRIVE_STRAIGHT;
float currentTargetDistanceM = 0.0; // Dynamic target distance for current step
float turnDirection = 1.0;          // +1.0 for Left, -1.0 for Right
float stepDistanceTraveled = 0.0;

float currentRampedSpeed = 0.0;
const float MAX_TARGET_SPEED_MS = 0.30;   // Cruise speed for straight driving (m/s)
const float MAX_TURN_SPEED_MS   = 0.15;   // Cruise speed for turning (m/s)
const float ACCEL_STEP_MS = 0.005;        // Acceleration increment per 20ms
const float DECEL_STEP_MS = 0.005;        // Deceleration decrement per 20ms

// --- Speed Measurement & Filtering ---
float rawActualLeftSpeed = 0.0;
float actualLeftSpeed = 0.0; 
float lastActualLeftSpeed = 0.0;

float rawActualRightSpeed = 0.0;
float actualRightSpeed = 0.0; 
float lastActualRightSpeed = 0.0;

// --- PID & Sync Gains ---
float Kp = 150.0;
float Ki = 30.0;
float Kd = 1.2;

const float K_SYNC_P = 2.0;
const float K_SYNC_I = 4.0;
float speedSyncErrorSum = 0.0;

float errorLeftSum = 0.0;
float errorRightSum = 0.0;
long prevLeftTicks = 0;
long prevRightTicks = 0;

ISR(PCINT1_vect) {
  static uint8_t lastPortC = 0;
  uint8_t currentPortC = PINC; 

  if ((currentPortC & (1 << PC3)) && !(lastPortC & (1 << PC3))) {
    if (currentPortC & (1 << PC2))
      leftEncoderTicks++;  
    else
      leftEncoderTicks--;  
  }

  if ((currentPortC & (1 << PC1)) && !(lastPortC & (1 << PC1))) {
    if (currentPortC & (1 << PC0))
      rightEncoderTicks--;  
    else
      rightEncoderTicks++;  
  }
  lastPortC = currentPortC;
}

void setupEncoders() {
  DDRC &= ~0b00111100;
  PORTC |= (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3);
  PCICR |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT9) | (1 << PCINT11);
}

void driveMotors(int leftPWM, int rightPWM) {
  int pwmL = constrain(abs(leftPWM), 0, 255);
  if (leftPWM > 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else if (leftPWM < 0) {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  }
  analogWrite(ENB, pwmL);

  int pwmR = constrain(abs(rightPWM), 0, 255);
  if (rightPWM > 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  } else if (rightPWM < 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  }
  analogWrite(ENA, pwmR);
}

// ==========================================================
// --- COMMAND INTERFACE FUNCTIONS ---
// ==========================================================

// Configures the robot to drive straight for a given distance in METERS
void handleDistance(float distanceInMeters) {
  currentMotionType = DRIVE_STRAIGHT;
  currentTargetDistanceM = distanceInMeters;
  stepDistanceTraveled = 0.0;
  currentState = ACCELERATING;
}

// Configures the robot to tank-turn by a given ANGLE in DEGREES
// direction: +1.0 = Left Turn, -1.0 = Right Turn
void turnAngle(float angleInDegrees, float direction = 1.0) {
  currentMotionType = TANK_TURN;
  turnDirection = direction;
  
  // Convert angle in degrees to wheel distance in meters: s = (pi * W * angle) / 360
  currentTargetDistanceM = (PI * WHEEL_BASE_M * abs(angleInDegrees)) / 360.0;
  stepDistanceTraveled = 0.0;
  currentState = ACCELERATING;
}

// ==========================================================
// --- MISSION SEQUENCER ---
// ==========================================================

uint8_t currentMissionStep = 0;

void executeMissionStep(uint8_t step) {
  switch (step) {
    case 0: handleDistance(2.0);          break; // Go 2 meters forward
    case 1: turnAngle(90.0, 1.0);         break; // Tank turn 90 deg Left
    case 2: handleDistance(1.0);          break; // Go 1 meter forward
    case 3: turnAngle(180.0, 1.0);        break; // Turn around 180 deg
    case 4: handleDistance(1.0);          break; // Go 1 meter forward
    case 5: turnAngle(90.0, 1.0);         break; // Tank turn 90 deg Left
    case 6: handleDistance(2.0);          break; // Go 2 meters forward back to start
    default:
      currentState = COMPLETED;
      break;
  }
}

// ==========================================================
// --- STATE MACHINE FUNCTIONS ---
// ==========================================================

float getStepMaxSpeed() {
  return (currentMotionType == TANK_TURN) ? MAX_TURN_SPEED_MS : MAX_TARGET_SPEED_MS;
}

bool isDecelerationNeeded() {
  float decel_m_s2 = DECEL_STEP_MS / (CONTROL_INTERVAL_MS / 1000.0);
  float stoppingDistance = (currentRampedSpeed * currentRampedSpeed) / (2.0 * decel_m_s2);
  float remainingDistance = currentTargetDistanceM - stepDistanceTraveled;
  return remainingDistance <= stoppingDistance;
}

void stateAccelerating(unsigned long now) {
  float maxSpeed = getStepMaxSpeed();

  if (isDecelerationNeeded()) {
    currentState = DECELERATING;
    errorLeftSum = 0.0;
    errorRightSum = 0.0;
    return;
  }

  if (currentRampedSpeed < maxSpeed) {
    currentRampedSpeed += ACCEL_STEP_MS;
    if (currentRampedSpeed >= maxSpeed) {
      currentRampedSpeed = maxSpeed;
      currentState = CRUISING;
    }
  }
}

void stateCruising(unsigned long now) {
  currentRampedSpeed = getStepMaxSpeed();

  if (isDecelerationNeeded()) {
    currentState = DECELERATING;
    errorLeftSum = 0.0;
    errorRightSum = 0.0;
  }
}

void stateDecelerating(unsigned long now) {
  if (currentRampedSpeed > 0.0) {
    currentRampedSpeed -= DECEL_STEP_MS;
    if (currentRampedSpeed <= 0.0) {
      currentRampedSpeed = 0.0;
      stepPauseStartTime = now;
      currentState = PAUSING;
    }
  }
}

void statePausing(unsigned long now) {
  currentRampedSpeed = 0.0;
  errorLeftSum = 0.0;
  errorRightSum = 0.0;
  speedSyncErrorSum = 0.0;

  if (now - stepPauseStartTime >= PAUSE_BETWEEN_STEPS_MS) {
    currentMissionStep++;
    executeMissionStep(currentMissionStep);
  }
}

void stateCompleted(unsigned long now) {
  currentRampedSpeed = 0.0;
}

void updateStateMachine(unsigned long now) {
  switch (currentState) {
    case ACCELERATING:  stateAccelerating(now);  break;
    case CRUISING:      stateCruising(now);      break;
    case DECELERATING:  stateDecelerating(now);  break;
    case PAUSING:       statePausing(now);       break;
    case COMPLETED:     stateCompleted(now);     break;
  }
}

void setup() {
  Serial.begin(115200);
  DDRB |= 0b00111111;
  setupEncoders();

  // Start the mission with Step 0
  executeMissionStep(0);
}

void loop() {
  unsigned long now = millis();

  if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
    float dt = (now - lastControlTime) / 1000.0;
    lastControlTime = now;

    noInterrupts();
    long currentLeftTicks = leftEncoderTicks;
    long currentRightTicks = rightEncoderTicks;
    interrupts();

    // 1. Calculate delta ticks and raw wheel speeds
    long deltaLeft = currentLeftTicks - prevLeftTicks;
    long deltaRight = currentRightTicks - prevRightTicks;
    prevLeftTicks = currentLeftTicks;
    prevRightTicks = currentRightTicks;

    rawActualLeftSpeed = (deltaLeft * METERS_PER_TICK) / dt;
    rawActualRightSpeed = (deltaRight * METERS_PER_TICK) / dt;

    // 2. Exponential Moving Average for speed
    constexpr float SPEED_FILTER_ALPHA = 0.15;
    actualLeftSpeed += SPEED_FILTER_ALPHA * (rawActualLeftSpeed - actualLeftSpeed);
    actualRightSpeed += SPEED_FILTER_ALPHA * (rawActualRightSpeed - actualRightSpeed);

    // 3. Integrate Traveled Distance for Current Step
    if (currentState != PAUSING && currentState != COMPLETED) {
      if (currentMotionType == DRIVE_STRAIGHT) {
        stepDistanceTraveled += ((deltaLeft + deltaRight) / 2.0) * METERS_PER_TICK;
      } else {
        // Tank turn: integrate absolute displacement since wheels turn in opposite directions
        stepDistanceTraveled += ((abs(deltaLeft) + abs(deltaRight)) / 2.0) * METERS_PER_TICK;
      }
    }

    // 4. Dispatch State Machine Execution
    updateStateMachine(now);

    // 5. Determine Target Speeds for Left and Right Wheels
    float targetLeftSpeed = 0.0;
    float targetRightSpeed = 0.0;

    if (currentState != PAUSING && currentState != COMPLETED) {
      if (currentMotionType == DRIVE_STRAIGHT) {
        targetLeftSpeed  = currentRampedSpeed;
        targetRightSpeed = currentRampedSpeed;
      } else {
        // Tank Turn: Opposite directions based on turn direction (+1 = Left, -1 = Right)
        targetLeftSpeed  = -turnDirection * currentRampedSpeed;
        targetRightSpeed =  turnDirection * currentRampedSpeed;
      }
    }

    // 6. Closed-Loop Speed PID Calculations
    float speedLeftError  = targetLeftSpeed - actualLeftSpeed;
    float speedRightError = targetRightSpeed - actualRightSpeed;

    errorLeftSum += speedLeftError * dt;
    errorLeftSum = constrain(errorLeftSum, -5.0, 5.0); 

    errorRightSum += speedRightError * dt;
    errorRightSum = constrain(errorRightSum, -5.0, 5.0); 

    float dActualLeftSpeed  = (actualLeftSpeed - lastActualLeftSpeed) / dt;
    float dActualRightSpeed = (actualRightSpeed - lastActualRightSpeed) / dt;
    lastActualLeftSpeed  = actualLeftSpeed;
    lastActualRightSpeed = actualRightSpeed;

    // Base Feedforward PWM
    float maxSpeed = getStepMaxSpeed();
    float baseLeftPWM  = (targetLeftSpeed / maxSpeed) * 240.0;
    float baseRightPWM = (targetRightSpeed / maxSpeed) * 240.0;

    float finalLeftPWM  = baseLeftPWM + (Kp * speedLeftError) + (Ki * errorLeftSum) - (Kd * dActualLeftSpeed);
    float finalRightPWM = baseRightPWM + (Kp * speedRightError) + (Ki * errorRightSum) - (Kd * dActualRightSpeed);

    // Cross-coupled PI Synchronization
    if (currentState != PAUSING && currentState != COMPLETED && currentRampedSpeed > 0.0) {
      float speedSyncError = (currentMotionType == DRIVE_STRAIGHT) 
        ? (actualLeftSpeed - actualRightSpeed)
        : (actualLeftSpeed + actualRightSpeed);

      speedSyncErrorSum += speedSyncError * dt;
      speedSyncErrorSum = constrain(speedSyncErrorSum, -0.5, 0.5); 

      float syncCorrection = (K_SYNC_P * speedSyncError) + (K_SYNC_I * speedSyncErrorSum);
      syncCorrection = constrain(syncCorrection, -40.0, 40.0); 

      finalLeftPWM  -= syncCorrection;
      finalRightPWM += syncCorrection;
    }

    // Motor Output Control
    if (currentState == PAUSING || currentState == COMPLETED) {
      driveMotors(0, 0);
    } else {
      driveMotors((int)finalLeftPWM, (int)finalRightPWM);
    }
  }

  // --- Serial Monitoring ---
  if (now - lastDebugPrintMS >= DEBUG_PRINT_INTERVAL_MS) {
    lastDebugPrintMS = now;

    Serial.print("Step: ");
    Serial.print(currentMissionStep);
    Serial.print(" | State: ");
    Serial.print(currentState);
    Serial.print(" | StepDist: ");
    Serial.print(stepDistanceTraveled);
    Serial.print(" / ");
    Serial.println(currentTargetDistanceM);
  }
}