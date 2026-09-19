#include "robotconfig.h"
#include <Arduino.h>

volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

// --- Timing Control ---
const unsigned long CONTROL_INTERVAL_MS = 20;     // 50 Hz Control Loop
const unsigned long DEBUG_PRINT_INTERVAL_MS = 100;

unsigned long lastControlTime = 0;
unsigned long lastDebugPrintMS = 0;

// --- Physical Robot Parameters ---
const float METERS_PER_TICK = (2.0 * PI * WHEEL_RADIUS) / TICKS_PER_REV;

// --- Distance & Ramping Settings (m/s) ---
const float TARGET_DISTANCE_M = 2.0;       // 2 Meters Target
float totalDistanceTraveled = 0.0;         // Integrated actual distance

float currentRampedSpeed = 0.0;
const float MAX_TARGET_SPEED_MS = 0.30;   // Maximum cruise speed in m/s
const float ACCEL_STEP_MS = 0.005;        // Acceleration increment per 20ms
const float DECEL_STEP_MS = 0.05;        // Deceleration decrement per 20ms

// --- Speed Measurement & Filtering ---
float rawActualLeftSpeed = 0.0;
float actualLeftSpeed = 0.0; 
float lastActualLeftSpeed = 0.0;

float rawActualRightSpeed = 0.0;
float actualRightSpeed = 0.0; 
float lastActualRightSpeed = 0.0;

// --- Motion States ---
// 0: Accelerating, 1: Holding/Cruising, 2: Decelerating, 3: Complete & Stopped
uint8_t runPhase = 0; 

// --- PID VALUES (Tuned for m/s scale) ---
float Kp = 150.0;  // Proportional Gain
float Ki = 30.0;   // Integral Gain
float Kd = 1.2;    // Derivative Gain

// Wheel synchronization
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

// --- Distance Trigger Logic ---
void handleDistance() {
  // Compute deceleration rate: 0.005 m/s per 0.02s loop = 0.25 m/s^2
  float decel_m_s2 = DECEL_STEP_MS / (CONTROL_INTERVAL_MS / 1000.0);
  
  // Kinematic stopping distance: d = v^2 / (2 * a)
  float stoppingDistance = (currentRampedSpeed * currentRampedSpeed) / (2.0 * decel_m_s2);
  float remainingDistance = TARGET_DISTANCE_M - totalDistanceTraveled;

  // Trigger deceleration phase once we reach the deceleration threshold
  if (remainingDistance <= stoppingDistance && (runPhase == 0 || runPhase == 1)) {
    runPhase = 2; // Move to Decelerating phase
    errorLeftSum = 0.0;
    errorRightSum = 0.0;
  }
}

// --- State Machine Functions ---
void handleAccelerating() {
  if (currentRampedSpeed < MAX_TARGET_SPEED_MS) {
    currentRampedSpeed += ACCEL_STEP_MS;
    if (currentRampedSpeed >= MAX_TARGET_SPEED_MS) {
      currentRampedSpeed = MAX_TARGET_SPEED_MS;
      runPhase = 1; // Holding/Cruising
    }
  }
}

void handleHolding() {
  currentRampedSpeed = MAX_TARGET_SPEED_MS;
}

void handleDecelerating() {
  if (currentRampedSpeed > 0.0) {
    currentRampedSpeed -= DECEL_STEP_MS;
    if (currentRampedSpeed <= 0.0) {
      currentRampedSpeed = 0.0;
      runPhase = 3; // Stopped
    }
  }
}

void handleStopped() {
  currentRampedSpeed = 0.0;
  errorLeftSum = 0.0;
  errorRightSum = 0.0;
}

void updateStateMachine() {
  switch (runPhase) {
    case 0:
      handleAccelerating();
      break;
    case 1:
      handleHolding();
      break;
    case 2:
      handleDecelerating();
      break;
    case 3:
      handleStopped();
      break;
  }
}

void setup() {
  Serial.begin(115200);
  DDRB |= 0b00111111;
  setupEncoders();
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

    // 1. Calculate delta ticks and raw speed in m/s
    long deltaLeft = currentLeftTicks - prevLeftTicks;
    long deltaRight = currentRightTicks - prevRightTicks;
    prevLeftTicks = currentLeftTicks;
    prevRightTicks = currentRightTicks;

    rawActualLeftSpeed = (deltaLeft * METERS_PER_TICK) / dt;
    rawActualRightSpeed = (deltaRight * METERS_PER_TICK) / dt;

    // 2. Distance Integration (Average of both wheels)
    if (runPhase != 3) {
      float deltaDistance = ((deltaLeft + deltaRight) / 2.0) * METERS_PER_TICK;
      totalDistanceTraveled += deltaDistance;
    }

    // 3. Exponential Moving Average for speed
    constexpr float SPEED_FILTER_ALPHA = 0.15;
    actualLeftSpeed += SPEED_FILTER_ALPHA * (rawActualLeftSpeed - actualLeftSpeed);
    actualRightSpeed += SPEED_FILTER_ALPHA * (rawActualRightSpeed - actualRightSpeed);

    // 4. Update distance check and motion states
    handleDistance();
    updateStateMachine();

    // 5. PID Speed Control Calculation
    float speedLeftError = currentRampedSpeed - actualLeftSpeed;
    float speedRightError = currentRampedSpeed - actualRightSpeed;

    errorLeftSum += speedLeftError * dt;
    errorLeftSum = constrain(errorLeftSum, -5.0, 5.0); 

    errorRightSum += speedRightError * dt;
    errorRightSum = constrain(errorRightSum, -5.0, 5.0); 

    // Derivative on Measurement
    float dActualLeftSpeed = (actualLeftSpeed - lastActualLeftSpeed) / dt;
    float dActualRightSpeed = (actualRightSpeed - lastActualRightSpeed) / dt;
    lastActualLeftSpeed = actualLeftSpeed;
    lastActualRightSpeed = actualRightSpeed;

    // 6. Feedforward Base PWM Mapping
    float basePWM = (currentRampedSpeed / MAX_TARGET_SPEED_MS) * 240.0;

    // Combined Feedforward + PID Output
    float finalLeftPWM = basePWM + (Kp * speedLeftError) + (Ki * errorLeftSum) - (Kd * dActualLeftSpeed);
    float finalRightPWM = basePWM + (Kp * speedRightError) + (Ki * errorRightSum) - (Kd * dActualRightSpeed);

    // Cross-coupled PI synchronization
    if (runPhase != 3 && currentRampedSpeed > 0.0) {
      float speedSyncError = actualLeftSpeed - actualRightSpeed;
      speedSyncErrorSum += speedSyncError * dt;
      speedSyncErrorSum = constrain(speedSyncErrorSum, -0.5, 0.5); 

      float syncCorrection = (K_SYNC_P * speedSyncError) + (K_SYNC_I * speedSyncErrorSum);
      syncCorrection = constrain(syncCorrection, -40.0, 40.0); 

      finalLeftPWM -= syncCorrection;
      finalRightPWM += syncCorrection;
    } else {
      speedSyncErrorSum = 0.0;
    }

    // Minimum PWM kick during deceleration phase
    if (runPhase == 2 && finalRightPWM > 0 && finalRightPWM < 35 && finalLeftPWM > 0 && finalLeftPWM < 35 && currentRampedSpeed > 0.01) {
      finalLeftPWM = 35;
      finalRightPWM = 35;
    }

    // Motor Output Control
    if (runPhase == 3) {
      driveMotors(0, 0); // Complete stop
    } else {
      driveMotors((int)finalLeftPWM*0.9, (int)finalRightPWM);
    }
  }

  // --- Serial Plotter Output ---
  if (now - lastDebugPrintMS >= DEBUG_PRINT_INTERVAL_MS) {
    lastDebugPrintMS = now;

    cli();
    long currentLeftTicks = leftEncoderTicks;
    long currentRightTicks = rightEncoderTicks;
    sei();

    Serial.print(currentLeftTicks);
    Serial.print(", ");
    Serial.println(currentRightTicks);
  }
}