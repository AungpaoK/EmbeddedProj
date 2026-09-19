#include "robotconfig.h"

uint8_t debugPrintIntervalMS = 20;
unsigned long lastDebugPrintMS = 0;

volatile long rightEncoderTicks = 0;
volatile long leftEncoderTicks = 0;

// Interrupt Service Routine for PORTC
ISR(PCINT1_vect) {
  static uint8_t lastPortC = 0;
  uint8_t currentPortC = PINC; // Read Port C input register

  // Left encoder: Phase A = A5/PC5, Phase B = A4/PC4.
  if ((currentPortC & (1 << PC5)) && !(lastPortC & (1 << PC5))) {
    if (currentPortC & (1 << PC4))
      leftEncoderTicks++;
    else
      leftEncoderTicks--;
  }

  // Right encoder: Phase A = A1/PC1, Phase B = A0/PC0.
  if ((currentPortC & (1 << PC1)) && !(lastPortC & (1 << PC1))) {
    if (currentPortC & (1 << PC0))
      rightEncoderTicks--;
    else
      rightEncoderTicks++;
  }
  lastPortC = currentPortC;
}

void setupEncoders() {
  DDRC &= ~0b00110011;
  PORTC |= (1 << PC0) | (1 << PC1) | (1 << PC4) | (1 << PC5);
  PCICR |= (1 << PCIE1);
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
    long currentLeftTicks = leftEncoderTicks;
    long currentRightTicks = rightEncoderTicks;
    sei();

    Serial.print(currentLeftTicks);
    Serial.print(", ");
    Serial.println(currentRightTicks);

  }
}
