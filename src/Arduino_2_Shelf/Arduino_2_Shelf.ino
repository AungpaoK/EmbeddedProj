/*
 * Arduino_2_Shelf.ino
 * =====================
 * Shelf & User Interface Controller — Arduino Uno R3 #2
 *
 * หน้าที่:
 *   - อ่านสถานะ IR Sensor บนชั้น 1 (D2) และชั้น 2 (D3)
 *   - อ่านปุ่มกด Keypad 4x4 Matrix
 *   - ตรวจจับปุ่ม Manual Override (D12)
 *   - ส่งผลลัพธ์ผ่าน Serial ไปยัง Raspberry Pi
 *   - รับคำสั่งแสดงผล LCD 16x2 (I2C) จาก Pi
 *
 * Serial Protocol:
 *   ส่ง:  IR:<shelf>,<status>\n   (shelf=1|2, status=1=มีอาหาร, 0=ไม่มี)
 *         KEY:<char>\n             (ตัวอักษรจาก Keypad)
 *         OVERRIDE\n               (ปุ่ม Manual Override ถูกกด)
 *   รับ:  LCD:<row>,<text>\n       (แสดงข้อความ บน LCD)
 *
 * Pin Map:
 *   IR Sensor ชั้น 1:  D2  (LOW = มีอาหาร — Active Low)
 *   IR Sensor ชั้น 2:  D3  (LOW = มีอาหาร — Active Low)
 *   Keypad Rows:        D4, D5, D6, D7
 *   Keypad Cols:        D8, D9, D10, D11
 *   Manual Override:    D13  (LOW = กด — Active Low, Internal Pull-up)
 *   LCD I2C:            A4 (SDA), A5 (SCL) — 0x27 default address
 *
 * Library ที่ต้องติดตั้ง:
 *   - LiquidCrystal_I2C  (ติดตั้งผ่าน Arduino Library Manager)
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ============================================================
// Pin Definitions
// ============================================================
const uint8_t IR_SHELF_1_PIN = 2;   // IR Sensor ชั้น 1 (Active LOW)
const uint8_t IR_SHELF_2_PIN = 3;   // IR Sensor ชั้น 2 (Active LOW)

// Keypad 4x4 Matrix
const uint8_t ROW_PINS[4] = {4, 5, 6, 7};   // Row 1–4
const uint8_t COL_PINS[4] = {8, 9, 10, 11}; // Col 1–4

const uint8_t OVERRIDE_PIN = 13;   // Manual Override Button (Active LOW)

// ============================================================
// Keypad Layout
// ============================================================
const char KEYMAP[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

// ============================================================
// LCD (I2C, address 0x27, 16 columns, 2 rows)
// ============================================================
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ============================================================
// State
// ============================================================
bool     lastIR1         = false;
bool     lastIR2         = false;
bool     lastOverride    = false;
uint32_t lastIRSendMs    = 0;
uint32_t lastKeyDebounce = 0;
char     lastKey         = 0;

const uint32_t IR_SEND_INTERVAL_MS  = 100;  // ส่งสถานะ IR ทุก 100ms
const uint32_t KEY_DEBOUNCE_MS      = 50;   // Debounce keypad

// ============================================================
// IR Sensor Reader
// ============================================================
bool readIR(uint8_t pin) {
    // Active LOW: LOW = มีอาหาร (วัตถุกั้นแสง)
    return (digitalRead(pin) == LOW);
}

void sendIRStatus(uint8_t shelf, bool hasFood) {
    Serial.print("IR:");
    Serial.print(shelf);
    Serial.print(",");
    Serial.println(hasFood ? 1 : 0);
}

// ============================================================
// Keypad Scanner (Non-blocking Matrix Scan)
// ============================================================
char scanKeypad() {
    for (uint8_t r = 0; r < 4; r++) {
        // ดึง Row ที่สแกนให้เป็น LOW, Row อื่น HIGH
        for (uint8_t i = 0; i < 4; i++) {
            digitalWrite(ROW_PINS[i], (i == r) ? LOW : HIGH);
        }
        delayMicroseconds(10);  // ให้สัญญาณ settle

        for (uint8_t c = 0; c < 4; c++) {
            if (digitalRead(COL_PINS[c]) == LOW) {
                // คืนค่า key ที่กด
                return KEYMAP[r][c];
            }
        }
    }
    // คืน Row ทั้งหมดเป็น HIGH (idle state)
    for (uint8_t i = 0; i < 4; i++) {
        digitalWrite(ROW_PINS[i], HIGH);
    }
    return 0;  // ไม่มีการกด
}

// ============================================================
// LCD Command Handler
// ============================================================
void handleLCDCommand(const String &line) {
    // Format: LCD:<row>,<text>
    if (!line.startsWith("LCD:")) return;

    int commaIdx = line.indexOf(',', 4);
    if (commaIdx < 0) return;

    int row  = line.substring(4, commaIdx).toInt();
    String text = line.substring(commaIdx + 1);

    if (row < 0 || row > 1) return;

    lcd.setCursor(0, row);
    // ล้างบรรทัดก่อนแล้วพิมพ์ใหม่
    lcd.print("                ");  // 16 spaces
    lcd.setCursor(0, row);
    lcd.print(text.substring(0, 16));
}

// ============================================================
// setup
// ============================================================
void setup() {
    Serial.begin(115200);

    // IR Pins — Input with Internal Pull-up
    pinMode(IR_SHELF_1_PIN, INPUT_PULLUP);
    pinMode(IR_SHELF_2_PIN, INPUT_PULLUP);

    // Override Button — Input with Internal Pull-up
    pinMode(OVERRIDE_PIN, INPUT_PULLUP);

    // Keypad Rows — Output (เริ่มต้น HIGH)
    for (uint8_t r = 0; r < 4; r++) {
        pinMode(ROW_PINS[r], OUTPUT);
        digitalWrite(ROW_PINS[r], HIGH);
    }
    // Keypad Cols — Input with Internal Pull-up
    for (uint8_t c = 0; c < 4; c++) {
        pinMode(COL_PINS[c], INPUT_PULLUP);
    }

    // LCD Initialization
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Delivery Robot");
    lcd.setCursor(0, 1);
    lcd.print("Ready...");

    Serial.println("STATUS:READY");
}

// ============================================================
// loop
// ============================================================
void loop() {
    uint32_t now = millis();

    // --- 1. Broadcast IR Sensor Status (ทุก 100ms) ---
    if (now - lastIRSendMs >= IR_SEND_INTERVAL_MS) {
        lastIRSendMs = now;

        bool ir1 = readIR(IR_SHELF_1_PIN);
        bool ir2 = readIR(IR_SHELF_2_PIN);

        // ส่งเฉพาะเมื่อสถานะเปลี่ยน หรือทุกรอบก็ได้ (Pi filter เอง)
        sendIRStatus(1, ir1);
        sendIRStatus(2, ir2);

        lastIR1 = ir1;
        lastIR2 = ir2;
    }

    // --- 2. Keypad Scan (พร้อม Debounce) ---
    char key = scanKeypad();
    if (key != 0 && key != lastKey) {
        // ตรวจว่าผ่าน debounce time
        if (now - lastKeyDebounce >= KEY_DEBOUNCE_MS) {
            lastKeyDebounce = now;
            Serial.print("KEY:");
            Serial.println(key);
        }
    }
    lastKey = key;

    // --- 3. Manual Override Button ---
    bool overridePressed = (digitalRead(OVERRIDE_PIN) == LOW);
    if (overridePressed && !lastOverride) {
        // Rising edge (กดใหม่)
        Serial.println("OVERRIDE");
    }
    lastOverride = overridePressed;

    // --- 4. Serial LCD Command from Pi ---
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) {
            handleLCDCommand(line);
        }
    }
}
