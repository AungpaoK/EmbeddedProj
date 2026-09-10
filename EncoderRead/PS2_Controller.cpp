#include "PS2_Controller.h"
#include "PS2X_lib.h"
namespace
{
    uint8_t ps2Transfer(uint8_t outgoingByte)
    {
        uint8_t incomingByte = 0;

        for (uint8_t bit = 0; bit < 8; bit++)
        {
            digitalWrite(PS2_CMD_PIN, (outgoingByte & 0x01) ? HIGH : LOW);
            outgoingByte >>= 1;

            digitalWrite(PS2_CLK_PIN, LOW);
            delayMicroseconds(8);

            if (digitalRead(PS2_DAT_PIN) == HIGH)
            {
                incomingByte |= (1 << bit);
            }

            digitalWrite(PS2_CLK_PIN, HIGH);
            delayMicroseconds(8);
        }

        return incomingByte;
    }

    // --- Analog filtering + calibration for joystick X (ps2_data[5]) ---
    const uint8_t FILTER_SIZE = 5;
    const uint8_t DEADZONE_THRESHOLD = 12; // adjust as needed

    uint8_t analogBuffer[FILTER_SIZE] = {128, 128, 128, 128, 128};
    uint8_t analogIndex = 0;
    uint8_t analogCount = 0; // how many valid samples in buffer (<= FILTER_SIZE)
    uint8_t analogCenter = 128;
    bool analogCalibrated = false;

    void addAnalogSample(uint8_t v)
    {
        analogBuffer[analogIndex] = v;
        analogIndex = (analogIndex + 1) % FILTER_SIZE;
        if (analogCount < FILTER_SIZE) analogCount++;
    }

    uint8_t getMedianAnalog()
    {
        uint8_t tmp[FILTER_SIZE];
        uint8_t n = analogCount;
        if (n == 0) return analogCenter;
        for (uint8_t i = 0; i < n; i++) tmp[i] = analogBuffer[i];

        // simple sort (bubble/insertion ok for small N)
        for (uint8_t i = 0; i < n - 1; i++)
        {
            for (uint8_t j = i + 1; j < n; j++)
            {
                if (tmp[j] < tmp[i])
                {
                    uint8_t t = tmp[i];
                    tmp[i] = tmp[j];
                    tmp[j] = t;
                }
            }
        }

        return tmp[n/2];
    }
}

void PS2_Init() // does not need to be called externally since PS2_ReadData handles initialization on first call
{
    pinMode(PS2_CMD_PIN, OUTPUT);
    pinMode(PS2_CLK_PIN, OUTPUT);
    pinMode(PS2_ATT_PIN, OUTPUT);
    pinMode(PS2_DAT_PIN, INPUT_PULLUP);

    digitalWrite(PS2_CMD_PIN, HIGH);
    digitalWrite(PS2_CLK_PIN, HIGH);
    digitalWrite(PS2_ATT_PIN, HIGH);

    // Quick calibration: read a few samples to establish analog center
    for (uint8_t i = 0; i < FILTER_SIZE; i++)
    {
        uint8_t tmp[6] = {0};
        PS2_ReadData(tmp);
        addAnalogSample(tmp[5]);
        delay(20);
    }
    analogCenter = getMedianAnalog();
    analogCalibrated = true;
}

void PS2_ReadData(uint8_t *ps2_data)
{
    digitalWrite(PS2_ATT_PIN, LOW);
    delayMicroseconds(10);

    ps2Transfer(0x01);
    ps2Transfer(0x42);
    ps2Transfer(0x00);

    for (uint8_t i = 0; i < 6; i++)
    {
        ps2_data[i] = ps2Transfer(0x00);
    }

    // add recent analog X sample into filter buffer
    addAnalogSample(ps2_data[5]);

    delayMicroseconds(10);
    digitalWrite(PS2_ATT_PIN, HIGH);
}

void print_debug(PS2_Status status_left, PS2_Status status_right, PS2_Status gripper_status)
{
    // Print raw analog values and D-pad/button states from PS2X
    Serial.print("ANALOG RX:");
    Serial.print(ps2x.Analog(PSS_RX));
    Serial.print("  LX:");
    Serial.print(ps2x.Analog(PSS_LX));
    Serial.print("  RY:");
    Serial.print(ps2x.Analog(PSS_RY));
    Serial.print("  LY:");
    Serial.print(ps2x.Analog(PSS_LY));

    Serial.print("  | D-PAD U:" ); Serial.print(ps2x.Button(PSB_PAD_UP) ? 1 : 0);
    Serial.print(" D:" ); Serial.print(ps2x.Button(PSB_PAD_DOWN) ? 1 : 0);
    Serial.print(" L:" ); Serial.print(ps2x.Button(PSB_PAD_LEFT) ? 1 : 0);
    Serial.print(" R:" ); Serial.print(ps2x.Button(PSB_PAD_RIGHT) ? 1 : 0);

    // Right-side face buttons: Triangle, Square, Circle, Cross
    Serial.print("  | TRI:" ); Serial.print(ps2x.Button(PSB_TRIANGLE) ? 1 : 0);
    Serial.print(" SQ:" ); Serial.print(ps2x.Button(PSB_SQUARE) ? 1 : 0);
    Serial.print(" CIR:" ); Serial.print(ps2x.Button(PSB_CIRCLE) ? 1 : 0);
    Serial.print(" X:" ); Serial.print(ps2x.Button(PSB_CROSS) ? 1 : 0);

   Serial.print("  | STATUS: ");
    Serial.print(" LEFT :");
    switch (status_left)
    {
    case FORWARD: Serial.print("FORWARD"); break;
    case BACKWARD: Serial.print("BACKWARD"); break;
    case LEFT: Serial.print("LEFT"); break;
    case RIGHT: Serial.print("RIGHT"); break;
    case FORWARD_LEFT: Serial.print("FORWARD_LEFT"); break;
    case FORWARD_RIGHT: Serial.print("FORWARD_RIGHT"); break;
    case BACKWARD_LEFT: Serial.print("BACKWARD_LEFT"); break;
    case BACKWARD_RIGHT: Serial.print("BACKWARD_RIGHT"); break;
    default: Serial.print("STOP"); break;
    }

    Serial.print("  | STATUS: ");
    Serial.print("  |  RIGHT :");
    switch (status_right)
    {
    case FORWARD: Serial.print("FORWARD"); break;
    case BACKWARD: Serial.print("BACKWARD"); break;
    case LEFT: Serial.print("LEFT"); break;
    case RIGHT: Serial.print("RIGHT"); break;
    case FORWARD_LEFT: Serial.print("FORWARD_LEFT"); break;
    case FORWARD_RIGHT: Serial.print("FORWARD_RIGHT"); break;
    case BACKWARD_LEFT: Serial.print("BACKWARD_LEFT"); break;
    case BACKWARD_RIGHT: Serial.print("BACKWARD_RIGHT"); break;
    default: Serial.print("CENTER"); break;
    }

    Serial.println();
}
