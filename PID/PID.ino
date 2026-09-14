// --- L298N Motor Pins ---
#define IN1 8    // Left Motor Direction 2
#define IN2 9    // Left Motor Direction 1
#define ENA 10   // Left Motor Speed PWM
#define ENB 11   // Right Motor Speed PWM
#define IN3 12   // Right Motor Direction 2
#define IN4 13   // Right Motor Direction 1

// --- Right Motor Encoder Pins (Port C) ---
#define RIGHT_ENC_A A1  // PC1 (PCINT9) - Phase A
#define RIGHT_ENC_B A0  // PC0 (PCINT8) - Phase B

// --- Left Motor Encoder Pins (Port C) ---
#define LEFT_ENC_A A5  // PC5 (PCINT13) - Phase A
#define LEFT_ENC_B A4  // PC4 (PCINT12) - Phase B

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
float rawActualSpeed = 0.0;
float actualSpeed = 0.0; 
float lastActualSpeed = 0.0;

// --- Motion States ---
uint8_t runPhase = 0; // 0: Accelerating, 1: Holding, 2: Decelerating, 3: Pause

// --- PID VALUES ---
float Kp = 150;    // Proportional Gain
float Ki = 15;    // Integral Gain
float Kd = 1;     // Derivative Gain

float errorSum = 0.0;
float lastSpeedError = 0.0;
long prevTicks = 0;

ISR(PCINT1_vect) {
  static uint8_t lastPortC = 0;
  uint8_t currentPortC = PINC;

  if ((currentPortC & (1 << PC1)) && !(lastPortC & (1 << PC1))) {
    if (currentPortC & (1 << PC0)) {
      rightEncoderTicks--;
    } else {
      rightEncoderTicks++;
    }
  }
  lastPortC = currentPortC;
}

void setupEncoders() {
  DDRC &= ~0b00110011;
  PORTC |= (1 << PC0) | (1 << PC1);

  PCICR |= (1 << PCIE1);     
  PCMSK1 |= (1 << PCINT9);
}

void driveMotors(int speedVal) {
  int pwm = constrain(abs(speedVal), 0, 255);

  // Left Motor
  if (speedVal > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else if (speedVal < 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  }
  analogWrite(ENA, pwm);

  // Right Motor
  if (speedVal > 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else if (speedVal < 0) {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  }
  analogWrite(ENB, pwm);
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
    long currentTicks = rightEncoderTicks;
    interrupts();

    // 1. Calculate raw actual speed
    rawActualSpeed = currentTicks - prevTicks;
    prevTicks = currentTicks;

    // 2. Exponential Moving Average
    actualSpeed = (0.7 * actualSpeed) + (0.3 * rawActualSpeed);

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
          errorSum = 0.0;             // Reset accumulated integral memory before ramp-down
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
        errorSum = 0.0;               
        if (now - runStartTime >= PAUSE_DURATION_MS) {
          runPhase = 0;               
        }
        break;
    }

    // 3. PID Speed Control Calculation
    targetSpeedTicks = currentRampedPWM * 0.117647; 
    float speedError = targetSpeedTicks - actualSpeed;

    errorSum += speedError * dt;
    errorSum = constrain(errorSum, -30.0, 30.0);

    // Derivative on Measurement
    float dActualSpeed = (actualSpeed - lastActualSpeed) / dt;
    lastActualSpeed = actualSpeed;

    float finalPWM = currentRampedPWM + (Kp * speedError) + (Ki * errorSum) - (Kd * dActualSpeed);

    if (runPhase == 2 && finalPWM > 0 && finalPWM < 35 && targetSpeedTicks > 1.0) {
      finalPWM = 35;
    }

    if (runPhase == 3) {
      driveMotors(0);
    } else {
      driveMotors((int)finalPWM);
    }
  }

  // --- Serial Plotter Output ---
  if (now - lastDebugPrintMS >= DEBUG_PRINT_INTERVAL_MS) {
    lastDebugPrintMS = now;

    Serial.print("TargetSpeed:");
    Serial.print(targetSpeedTicks);
    Serial.print(" ");
    Serial.print("ActualSpeed:");
    Serial.println(actualSpeed);
  }
}