#include "robotconfig.h"

uint8_t debugPrintIntervalMS = 20;
unsigned long lastDebugPrintMS = 0;

volatile long rightEncoderTicks = 0;
volatile long leftEncoderTicks = 0;

// Interrupt Service Routine for PORTC
ISR(PCINT1_vect) {
  static uint8_t lastPortC = 0;
  uint8_t currentPortC = PINC; // Read Port C input register

  // Detect rising edge on Phase A
  if ((currentPortC & (1 << PC3)) && !(lastPortC & (1 << PC3))) {
    // Check Phase B to determine rotation direction
    if (currentPortC & (1 << PC2)) {
      leftEncoderTicks++;  // Forward
    } else {
      leftEncoderTicks--;  // Reverse
    }
  }

  // Detect rising edge on Phase A
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
  DDRC &= ~0b00111100;
  PORTC |= (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3);
  PCICR |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT9) | (1 << PCINT11);
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
    long currentLeftTicks = leftEncoderTicks;
    long currentRightTicks = rightEncoderTicks;
    sei();

    Serial.print(currentLeftTicks);
    Serial.print(", ");
    Serial.println(currentRightTicks);

  }
}
