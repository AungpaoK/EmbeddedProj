// --- L298N Motor Pins (Pins 8 to 13) ---
#define IN1 8    // Left Motor Direction 1
#define IN2 9    // Left Motor Direction 2
#define ENA 10   // Left Motor Speed PWM
#define ENB 11   // Right Motor Speed PWM
#define IN3 12   // Right Motor Direction 1
#define IN4 13   // Right Motor Direction 2

// --- Right Motor Encoder Pins ---
#define RIGHT_ENC_A A1 // PC1 (PCINT9)  - Phase A
#define RIGHT_ENC_B A0 // PC0 (PCINT8)  - Phase B

// --- Left Motor Encoder Pins ---
#define LEFT_ENC_A A5  // PC5 (PCINT13) - Phase A
#define LEFT_ENC_B A4  // PC4 (PCINT12) - Phase B

uint8_t debugPrintIntervalMS = 25;
unsigned long lastDebugPrintMS = 0;

volatile long rightEncoderTicks = 0;
volatile long leftEncoderTicks = 0;

// Interrupt Service Routine for PORTC
ISR(PCINT1_vect) {
  static uint8_t lastPortC = 0;
  uint8_t currentPortC = PINC; // Read Port C input register

  // Detect rising edge on Phase A (A4 / PC4)
  if ((currentPortC & (1 << PC4)) && !(lastPortC & (1 << PC4))) {
    // Check Phase B to determine rotation direction
    if (currentPortC & (1 << PC5)) {
      leftEncoderTicks++;  // Forward
    } else {
      leftEncoderTicks--;  // Reverse
    }
  }

  // Detect rising edge on Phase A (A4 / PC4)
  if ((currentPortC & (1 << PC0)) && !(lastPortC & (1 << PC0))) {
    // Check Phase B to determine rotation direction
    if (currentPortC & (1 << PC1)) {
      rightEncoderTicks--;  // Forward
    } else {
      rightEncoderTicks++;  // Reverse
    }
  }
  lastPortC = currentPortC;
}

void setupEncoders() {
 // Set A0, A1, A4, and A5 as INPUTs
  DDRC &= ~0b00110011;
  
  PORTC |= (1 << PC0) | (1 << PC1) | (1 << PC4) | (1 << PC5);

  // Enable Pin Change Interrupts on PORTC
  PCICR |= (1 << PCIE1);
  
  // Enable interrupt masks for Phase A on both motors (PCINT9 = A1, PCINT13 = A5)
  PCMSK1 |= (1 << PCINT9) | (1 << PCINT13);
}

void setup()
{
  Serial.begin(115200);

  // Initialize L298N pins as OUTPUTs
  DDRB |= 0b00111111;
  
  setupEncoders();
  sei();
}

void loop()
{
  unsigned long now = millis();

  if (now - lastDebugPrintMS >= debugPrintIntervalMS) {
    lastDebugPrintMS = now;

    cli();
    long currentRightTicks = rightEncoderTicks;
    long currentLeftTicks = leftEncoderTicks;
    sei();

    Serial.print("Right Encoder Ticks: ");
    Serial.println(currentRightTicks);

    Serial.print("Left Encoder Ticks: ");
    Serial.println(currentLeftTicks);
  }
}