// --- L298N Motor Pin Definitions ---
#define IN1 8    // Left Motor Direction 2
#define IN2 9    // Left Motor Direction 1
#define ENA 10   // Left Motor Speed PWM
#define ENB 11   // Right Motor Speed PWM
#define IN3 12   // Right Motor Direction 2
#define IN4 13   // Right Motor Direction 1

void driveMotors(int leftPWM, int rightPWM) {
  // --- Left Motor ---
  int pwmL = constrain(abs(leftPWM), 0, 255);
  if (leftPWM > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else if (leftPWM < 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  }
  analogWrite(ENA, pwmL);

  // --- Right Motor ---
  int pwmR = constrain(abs(rightPWM), 0, 255);
  if (rightPWM > 0) {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); // Inverted direction for right motor
  } else if (rightPWM < 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  }
  analogWrite(ENB, pwmR);
}

// --- Diagnostic Test Function ---
void testL298N() {
  Serial.println("--- Starting L298N Motor Diagnostics ---");

  // 1. Forward Test
  Serial.println("1. Moving FORWARD (Speed: 180)");
  driveMotors(180, 180);
  delay(2000);

  // 2. Reverse Test
  Serial.println("2. Moving REVERSE (Speed: 180)");
  driveMotors(-180, -180);
  delay(2000);

  // 3. Spin Left
  Serial.println("3. Turning LEFT");
  driveMotors(-180, 180);
  delay(1500);

  // 4. Spin Right
  Serial.println("4. Turning RIGHT");
  driveMotors(180, -180);
  delay(1500);

  // 5. Stop
  Serial.println("5. BRAKE / STOP");
  driveMotors(0, 0);
  Serial.println("--- Diagnostic Test Complete ---");
}

void setup() {
  Serial.begin(115200);
  
  // Set Pins 8-13 as OUTPUTs using Register B or pinMode
  DDRB |= 0b00111111;

  // Run the motor test sequence once on power-up
  testL298N();
}

void loop() {
  // Motors remain stopped in loop after test completes
}