// --- L298N Motor Pins ---
#define IN1 8    // Left Motor Direction 2
#define IN2 9    // Left Motor Direction 1
#define ENA 10   // Left Motor Speed PWM
#define ENB 11   // Right Motor Speed PWM
#define IN3 12   // Right Motor Direction 2
#define IN4 7   // Right Motor Direction 1

// --- Right Motor Encoder Pins (Port C) ---
#define RIGHT_ENC_A A1  // PC1 (PCINT9) - Phase A
#define RIGHT_ENC_B A0  // PC0 (PCINT8) - Phase B

// --- Left Motor Encoder Pins (Port C) ---
#define LEFT_ENC_A A5  // PC5 (PCINT13) - Phase A
#define LEFT_ENC_B A4  // PC4 (PCINT12) - Phase B

volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

// --- Timing Control ---
const unsigned long CONTROL_INTERVAL_MS = 20;     // 50 Hz Control Loop
const unsigned long DEBUG_PRINT_INTERVAL_MS = 100;
const unsigned long HOLD_DURATION_MS = 25000;     // Hold speed
const unsigned long PAUSE_DURATION_MS = 1000;     // Pause between cycles

unsigned long lastControlTime = 0;
unsigned long lastDebugPrintMS = 0;
unsigned long runStartTime = 0;

// --- Speed & Acceleration Settings ---
float currentRampedPWM = 0.0;
const float MAX_TARGET_PWM = 255.0;  // SET PWM TARGET
const float ACCEL_STEP = 3.0;        // Acceleration increment
const float DECEL_STEP = 3.0;        // Deceleration decrement

// --- Speed Measurement & Filtering ---
float targetSpeedTicks = 0.0;

float rawActualLeftSpeed = 0.0;
float actualLeftSpeed = 0.0; 
float lastActualLeftSpeed = 0.0;

float rawActualRightSpeed = 0.0;
float actualRightSpeed = 0.0; 
float lastActualRightSpeed = 0.0;

// --- Motion States ---
uint8_t runPhase = 0; // 0: Accelerating, 1: Holding, 2: Decelerating, 3: Pause

// --- PID VALUES ---
float Kp = 0;    // Proportional Gain
float Ki = 0;    // Integral Gain
float Kd = 0;     // Derivative Gain

// Wheel synchronization: slow the faster wheel and assist the slower wheel.
const float K_SYNC_P = 2.0;
const float K_SYNC_I = 4.0;
float speedSyncErrorSum = 0.0;

float errorLeftSum = 0.0;
float errorRightSum = 0.0;
float lastSpeedError = 0.0;
long prevLeftTicks = 0;
long prevRightTicks = 0;

ISR(PCINT1_vect) {
  static uint8_t lastPortC = 0;
  uint8_t currentPortC = PINC; // Read Port C input register

  // Left encoder: Phase A = A5/PC5, Phase B = A4/PC4.
  if ((currentPortC & (1 << PC5)) && !(lastPortC & (1 << PC5))) {
    if (currentPortC & (1 << PC4))
      leftEncoderTicks++;  // Forward
    else
      leftEncoderTicks--;  // Reverse
  }

  // Right encoder: Phase A = A1/PC1, Phase B = A0/PC0.
  if ((currentPortC & (1 << PC1)) && !(lastPortC & (1 << PC1))) {
    if (currentPortC & (1 << PC0))
      rightEncoderTicks--;  // Reverse
    else
      rightEncoderTicks++;  // Forward
  }
  lastPortC = currentPortC;
}

void setupEncoders() {
  // Encoder pins A0, A1, A4 and A5 are inputs with pull-ups.
  DDRC &= ~0b00110011;
  PORTC |= (1 << PC0) | (1 << PC1) | (1 << PC4) | (1 << PC5);
  PCICR |= (1 << PCIE1);
  // Interrupt on Phase A: right A1/PCINT9 and left A5/PCINT13.
  PCMSK1 |= (1 << PCINT9) | (1 << PCINT13);
}

void driveMotors(int leftPWM, int rightPWM) {
  // --- Left Motor ---
  int pwmL = constrain(abs(leftPWM), 0, 255);
  if (leftPWM > 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else if (leftPWM < 0) {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  }
  analogWrite(ENA, pwmL);

  // --- Right Motor ---
  int pwmR = constrain(abs(rightPWM), 0, 255);
  if (rightPWM > 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  } else if (rightPWM < 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  }
  analogWrite(ENB, pwmR);
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
    long currentRightTicks = rightEncoderTicks;
    long currentLeftTicks = leftEncoderTicks;
    interrupts();

    // 1. Calculate raw actual speed
    rawActualLeftSpeed = currentLeftTicks - prevLeftTicks;
    prevLeftTicks = currentLeftTicks;
    rawActualRightSpeed = currentRightTicks - prevRightTicks;
    prevRightTicks = currentRightTicks;

    // 2. Exponential Moving Average. The result remains a speed measured in
    // ticks per control interval instead of accumulating into total ticks.
    constexpr float SPEED_FILTER_ALPHA = 0.15;
    actualLeftSpeed += SPEED_FILTER_ALPHA *
                       (rawActualLeftSpeed - actualLeftSpeed);
    actualRightSpeed += SPEED_FILTER_ALPHA *
                        (rawActualRightSpeed - actualRightSpeed);

    // --- State Machine ---
    switch (runPhase) {
      case 0: // ACCELERATING TO 100 PWM
        if (currentRampedPWM < MAX_TARGET_PWM) {
          currentRampedPWM += ACCEL_STEP;
          if (currentRampedPWM >= MAX_TARGET_PWM) {
            currentRampedPWM = MAX_TARGET_PWM;
            runPhase = 1;               
            runStartTime = now;         
          }
        }
        break;

      case 1: // HOLDING AT 100 PWM FOR 15 SECONDS
        currentRampedPWM = MAX_TARGET_PWM;
        if (now - runStartTime >= HOLD_DURATION_MS) {
          runPhase = 2;               // Switch to Deceleration Phase
          errorLeftSum = 0.0;             // Reset accumulated integral memory before ramp-down
          errorRightSum = 0.0;             // Reset accumulated integral memory before ramp-down
        }
        break;

      case 2: // DECELERATING TO 0 PWM
        if (currentRampedPWM > 0.0) {
          currentRampedPWM -= DECEL_STEP;
          if (currentRampedPWM <= 0.0) {
            currentRampedPWM = 0.0;
            runPhase = 3;             
            runStartTime = now;       
          }
        }
        break;

      case 3: // PAUSE FOR 1 SECOND BEFORE REPEATING
        currentRampedPWM = 0.0;
        errorLeftSum = 0.0;               
        errorRightSum = 0.0;               
        if (now - runStartTime >= PAUSE_DURATION_MS) {
          runPhase = 0;               
        }
        break;
    }

    // 3. PID Speed Control Calculation
    targetSpeedTicks = currentRampedPWM * 0.117647; 
    float speedLeftError = targetSpeedTicks - actualLeftSpeed;
    float speedRightError = targetSpeedTicks - actualRightSpeed;

    errorLeftSum += speedLeftError * dt;
    errorLeftSum = constrain(errorLeftSum, -30.0, 30.0);

    errorRightSum += speedRightError * dt;
    errorRightSum = constrain(errorRightSum, -30.0, 30.0);

    // Derivative on Measurement
    float dActualLeftSpeed = (actualLeftSpeed - lastActualLeftSpeed) / dt;
    float dActualRightSpeed = (actualRightSpeed - lastActualRightSpeed) / dt;
    lastActualLeftSpeed = actualLeftSpeed;
    lastActualRightSpeed = actualRightSpeed;

    float finalLeftPWM = currentRampedPWM + (Kp * speedLeftError) + (Ki * errorLeftSum) - (Kd * dActualLeftSpeed);
    float finalRightPWM = currentRampedPWM + (Kp * speedRightError) + (Ki * errorRightSum) - (Kd * dActualRightSpeed);

    // Cross-coupled PI synchronization. A positive error means the left wheel
    // is faster, so reduce left PWM and increase right PWM by the same amount.
    if (runPhase != 3 && currentRampedPWM > 0.0) {
      float speedSyncError = actualLeftSpeed - actualRightSpeed;
      speedSyncErrorSum += speedSyncError * dt;
      speedSyncErrorSum = constrain(speedSyncErrorSum, -15.0, 15.0);

      float syncCorrection =
          (K_SYNC_P * speedSyncError) + (K_SYNC_I * speedSyncErrorSum);
      syncCorrection = constrain(syncCorrection, -60.0, 60.0);

      finalLeftPWM -= syncCorrection;
      finalRightPWM += syncCorrection;
    } else {
      speedSyncErrorSum = 0.0;
    }

    if (runPhase == 2 && finalRightPWM > 0 && finalRightPWM < 35 && finalLeftPWM > 0 && finalLeftPWM < 35 && targetSpeedTicks > 1.0) {
      finalLeftPWM = 35;
      finalRightPWM = 35;
    }

    if (runPhase == 3) {
      driveMotors(0, 0);
    } else {
      driveMotors((int)finalLeftPWM, (int)finalRightPWM);
    }
  }

  // --- Serial Plotter Output ---
  if (now - lastDebugPrintMS >= DEBUG_PRINT_INTERVAL_MS) {
    lastDebugPrintMS = now;

    cli();
    long currentLeftTicks = leftEncoderTicks;
    long currentRightTicks = rightEncoderTicks;
    sei();

    Serial.print("TargetSpeed:");
    Serial.print(targetSpeedTicks);
    Serial.print(" ");
    Serial.print("ActualLeftSpeed:");
    Serial.print(actualLeftSpeed);
    Serial.print(" ");
    Serial.print("ActualRightSpeed:");
    Serial.println(actualRightSpeed);
    Serial.print(currentLeftTicks);
    Serial.print(", ");
    Serial.println(currentRightTicks);
  }
}
