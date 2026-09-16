#include "PS2X_lib.h"
#include "PS2_Controller.h"
#include "robotconfig.h"

uint8_t ps2_data[6];
PS2_Status status_left = STOP;
PS2_Status status_right = STOP;

// PS2X library instance
PS2X ps2x;

// Deadzone for analog sticks (128 +/- 12 is considered idle)
const uint8_t PS2_DEADZONE = 12;

const unsigned long PS2_POLL_INTERVAL_MS = 15;
const unsigned long DEBUG_PRINT_INTERVAL_MS = 250;

unsigned long last_ps2_poll_ms = 0;
unsigned long last_debug_print_ms = 0;
unsigned long current_time = 0;

// Helper function to drive individual left motor
void drive_left_motor(int y_axis) {
  if (y_axis < -PS2_DEADZONE) {
    // Forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, map(abs(y_axis), PS2_DEADZONE, 128, 100, 255));
  } else if (y_axis > PS2_DEADZONE) {
    // Backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, map(abs(y_axis), PS2_DEADZONE, 128, 100, 255));
  } else {
    // Stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

// Fixed helper function to drive individual right motor (Inverted)
void drive_right_motor(int y_axis) {
  if (y_axis < -PS2_DEADZONE) {
    // Forward (Inverted logic applied)
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, map(abs(y_axis), PS2_DEADZONE, 128, 100, 255));
  } else if (y_axis > PS2_DEADZONE) {
    // Backward (Inverted logic applied)
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, map(abs(y_axis), PS2_DEADZONE, 128, 100, 255));
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
    drive_left_motor(0);
    drive_right_motor(0);

    ps2x.config_gamepad(PS2_CLK_PIN, PS2_CMD_PIN, PS2_ATT_PIN, PS2_DAT_PIN, true, true);
}

void loop()
{
    unsigned long now = millis();

    if (now - last_ps2_poll_ms >= PS2_POLL_INTERVAL_MS)
    {
        last_ps2_poll_ms = now;
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
        if (raw_ly == 255 && raw_ry == 255 && raw_lx == 255 && raw_rx == 255)
        {
            left_y = 0;
            right_y = 0;
        }
        // Independently drive left wheel with left stick & right wheel with right stick
        drive_left_motor(left_y);
        drive_right_motor(right_y);
    }
  current_time = millis();
  if (current_time - last_debug_print_ms >= 100) {
    Serial.print("Encoder Phase A: ");
    Serial.println(analogRead(A5));
    Serial.print("Encoder Phase B: ");
    Serial.println(analogRead(A4));
    last_debug_print_ms = current_time;
  }
}
