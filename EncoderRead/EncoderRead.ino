#include "PS2X_lib.h"
#include "PS2_Controller.h"

// --- L298N Motor Pins (Pins 8 to 13) ---
#define ENA 10   // Left Motor Speed PWM
#define IN1 8    // Left Motor Direction 1
#define IN2 9    // Left Motor Direction 2
#define IN3 12   // Right Motor Direction 1
#define IN4 13   // Right Motor Direction 2
#define ENB 11   // Right Motor Speed PWM

// --- Right Motor Encoder Pins ---
#define RIGHT_ENC_A A4  // PC4 (PCINT12) - Phase A
#define RIGHT_ENC_B A5  // PC5 (PCINT13) - Phase B

// --- Left Motor Encoder Pins ---
#define LEFT_ENC_A 2  // PD2 (INT0) - Phase A
#define LEFT_ENC_B 3  // PD3 (INT1) - Phase B

volatile long rightEncoderTicks = 0;
volatile long leftEncoderTicks = 0;

uint8_t ps2_data[6];
PS2_Status statusLeft = STOP;
PS2_Status statusRight = STOP;

// PS2X library instance
PS2X ps2x;

// Deadzone for analog sticks (128 +/- 12 is considered idle)
const uint8_t PS2_DEADZONE = 12;

const unsigned long PS2_POLL_INTERVAL_MS = 15;
const unsigned long DEBUG_PRINT_INTERVAL_MS = 250;

unsigned long lastPS2Poll = 0;
unsigned long lastDebugPrintMS = 0;

// Interrupt Service Routine for PORTC
ISR(PCINT1_vect) {
  static uint8_t lastPortC = 0;
  uint8_t currentPortC = PINC; // Read Port C input register

  // Detect rising edge on Phase A (A4 / PC4)
  if ((currentPortC & (1 << PC4)) && !(lastPortC & (1 << PC4))) {
    // Check Phase B (A5 / PC5) to determine rotation direction
    if (currentPortC & (1 << PC5)) {
      rightEncoderTicks++;  // Forward
    } else {
      rightEncoderTicks--;  // Reverse
    }
  }

  lastPortC = currentPortC;
}

// ISR for Left Encoder (Hardware Interrupt 0 / Pin 2)
ISR(INT0_vect) {
  // Read Phase B (PD3 / Pin 3) directly from register
  if (PIND & (1 << PD3)) {
    leftEncoderTicks++;  // Forward
  } else {
    leftEncoderTicks--;  // Reverse
  }
}

void setupEncoders() {
  // Set A4 (PC4) and A5 (PC5) as INPUTs
  DDRC &= ~((1 << PC4) | (1 << PC5));
  PORTC |= (1 << PC4) | (1 << PC5);

  // Enable Pin Change Interrupts on PORTC
  PCICR |= (1 << PCIE1);     // Enable PCINT1 interrupt group
  PCMSK1 |= (1 << PCINT12);  // Enable interrupt trigger on pin A4 (PCINT12)

  // --- Left Encoder (Pins 2 / 3) Setup ---
  DDRD &= ~((1 << PD2) | (1 << PD3));  // Inputs
  PORTD |= (1 << PD2) | (1 << PD3);   // Internal Pull-ups

  // Configure INT0 (Pin 2) to trigger on RISING edge
  EICRA |= (1 << ISC01) | (1 << ISC00);
  EIMSK |= (1 << INT0);                // Enable INT0 interrupt
}

// Helper function to drive individual left motor
void driveLeftMotor(int YAxis) {
  if (YAxis < -PS2_DEADZONE) {
    // Forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, map(abs(YAxis), PS2_DEADZONE, 128, 100, 255));
  } else if (YAxis > PS2_DEADZONE) {
    // Backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, map(abs(YAxis), PS2_DEADZONE, 128, 100, 255));
  } else {
    // Stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

// Fixed helper function to drive individual right motor (Inverted)
void driveRightMotor(int YAxis) {
  if (YAxis < -PS2_DEADZONE) {
    // Forward (Inverted logic applied)
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, map(abs(YAxis), PS2_DEADZONE, 128, 100, 255));
  } else if (YAxis > PS2_DEADZONE) {
    // Backward (Inverted logic applied)
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, map(abs(YAxis), PS2_DEADZONE, 128, 100, 255));
  } else {
    // Stop
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void setup()
{
  Serial.begin(115200);

  // Initialize L298N pins as OUTPUTs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Stop motors on startup
  driveLeftMotor(0);
  driveRightMotor(0);

  // Initialize Port C and Port D Pin Change Interrupts for Right Encoder
  setupEncoders();
  sei();

  ps2x.config_gamepad(PS2_CLK_PIN, PS2_CMD_PIN, PS2_ATT_PIN, PS2_DAT_PIN, true, true);
}

void loop()
{
  unsigned long now = millis();

  if (now - lastPS2Poll >= PS2_POLL_INTERVAL_MS){
    lastPS2Poll = now;
    ps2x.read_gamepad();
    
    // Read raw stick values (Center is ~128)
    uint8_t raw_ly = ps2x.Analog(PSS_LY);
    uint8_t raw_ry = ps2x.Analog(PSS_RY);
    uint8_t raw_lx = ps2x.Analog(PSS_LX);
    uint8_t raw_rx = ps2x.Analog(PSS_RX);
    
    // Convert to offset range: -128 (Full Forward) to +127 (Full Backward)
    int left_y = (int)raw_ly - 128;
    int right_y = (int)raw_ry - 128;

    // A disconnected wireless receiver typically returns 255 for ALL 4 analog axes simultaneously
    if (raw_ly == 255 && raw_ry == 255 && raw_lx == 255 && raw_rx == 255){
      left_y = 0;
      right_y = 0;
    }
    
    // Independently drive left wheel with left stick & right wheel with right stick
    driveLeftMotor(left_y);
    driveRightMotor(right_y);
  }

  if (now - lastDebugPrintMS >= DEBUG_PRINT_INTERVAL_MS) {
    lastDebugPrintMS = now;

    // Safely retrieve atomic 32-bit encoder value
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
