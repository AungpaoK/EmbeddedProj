/*
 * Arduino_1_Motion.ino
 * ======================
 * Motion & Drive Controller — Arduino Uno R3 #1
 *
 * หน้าที่:
 *   - รับคำสั่ง Serial จาก Raspberry Pi: FORWARD:<m>  TURN:<deg>  STOP
 *   - ขับมอเตอร์ด้วย Dual PID Speed Control + Wheel Sync (50 Hz loop)
 *   - ส่ง ENCODER:L,R ทุก 100ms ให้ Pi คำนวณ Odometry
 *   - ส่ง STATUS:DONE เมื่อทำคำสั่งเสร็จ, STATUS:ERROR เมื่อเกิดปัญหา
 *
 * Serial Protocol:
 *   รับ:  FORWARD:<distance_m>\n  |  TURN:<degrees>\n  |  STOP\n
 *         LIGHTTEST:LEFT\n  |  LIGHTTEST:RIGHT\n  |  LIGHTTEST:OFF\n
 *   ส่ง:  STATUS:DONE\n  |  STATUS:ERROR\n  |  ENCODER:<L>,<R>\n
 *
 * Pin Map (อ้างอิง robotconfig.h):
 *   D7=IN4, D8=IN1, D9=IN2, D10=ENA, D11=ENB, D12=IN3
 *   A0=RIGHT_ENC_A, A1=RIGHT_ENC_B, A3=LEFT_ENC_A, A2=LEFT_ENC_B
 */

#include "robotconfig.h"
#include "DisplayConfig.h"
#include "Arduino.h"
#include "TurnIndicator.h"

// ============================================================
// Encoder Sign Convention
// ============================================================
const bool LEFT_ENC_INVERT  = false;
const bool RIGHT_ENC_INVERT = true;  // Motor ขวาต่อสลับขั้ว

volatile long leftEncoderTicks  = 0;
volatile long rightEncoderTicks = 0;

// ============================================================
// Physical & Motion Constants
// ============================================================
const float METERS_PER_PULSE = (2.0 * 3.14159265 * WHEEL_RADIUS) / TICKS_PER_REV;

// N = (W/2) * π * (TPR / (2π*R))  — Ticks สำหรับการหมุน 180°
const float TICKS_PER_DEGREE =
    ((WHEEL_BASE / 2.0) * PI * (TICKS_PER_REV / (2.0 * 3.14159265 * WHEEL_RADIUS))) / 180.0;

const unsigned long CONTROL_INTERVAL_MS  = 20;    // 50 Hz PID loop
const unsigned long ENCODER_PRINT_MS     = 100;   // ส่ง Encoder ทุก 100ms

const float CRUISE_SPEED_MPS  = 0.25;
const float TURN_SPEED_MPS    = 0.15;
const float ACCEL_STEP_MPS    = 0.005;
const float DECEL_DIST_METERS = 0.20;
const float MIN_DRIVE_SPEED   = 0.04;   // ความเร็วขั้นต่ำเพื่อไม่ให้มอเตอร์ฝืด

// Automatic breakaway assist: if a wheel receives a real speed target but
// produces no encoder ticks, add bounded PWM in small steps. This handles
// static friction without running a stalled motor at full power indefinitely.
const float STALL_MIN_TARGET_MPS          = 0.05f;
const unsigned long STALL_DETECT_MS       = 350;
const unsigned long STALL_PWM_STEP_MS     = 250;
const unsigned long STALL_ABORT_MS        = 3000;
const int STALL_PWM_STEP                 = 20;
const int STALL_PWM_MAX_BOOST            = 120;

float K_sync = 1.5;   // Cross-coupling Sync Gain

// ============================================================
// PID Controller
// ============================================================
struct PIDController {
    float Kp = 150.0;
    float Ki = 10.0;
    float Kd = 1.2;

    float errorSum      = 0.0;
    float lastSpeed     = 0.0;
    float actualSpeed   = 0.0;
    long  prevTicks     = 0;
    unsigned long lastEncoderMotionMs = 0;
    unsigned long lastStallBoostMs = 0;
    int stallBoostPwm = 0;
    int lastTargetSign = 0;
    bool stallFault = false;

    void reset() {
        errorSum    = 0.0;
        lastSpeed   = 0.0;
        actualSpeed = 0.0;
        lastEncoderMotionMs = millis();
        lastStallBoostMs = lastEncoderMotionMs;
        stallBoostPwm = 0;
        lastTargetSign = 0;
        stallFault = false;
    }

    float compute(long currentTicks, float targetMPS, float dt) {
        long  delta   = currentTicks - prevTicks;
        prevTicks     = currentTicks;

        float rawSpeed = (delta * METERS_PER_PULSE) / dt;
        actualSpeed    = 0.85f * actualSpeed + 0.15f * rawSpeed;

        float err = targetMPS - actualSpeed;
        errorSum  = constrain(errorSum + err * dt, -2.0f, 2.0f);

        float dSpeed = (actualSpeed - lastSpeed) / dt;
        lastSpeed = actualSpeed;

        float ff  = (targetMPS / CRUISE_SPEED_MPS) * 200.0f;
        float out = ff + Kp * err + Ki * errorSum - Kd * dSpeed;

        if (abs(targetMPS) > 0.01f && abs(out) < 35.0f) {
            out = (targetMPS > 0) ? 35.0f : -35.0f;
        }

        unsigned long nowMs = millis();
        int targetSign = (targetMPS > STALL_MIN_TARGET_MPS) ? 1 :
                         (targetMPS < -STALL_MIN_TARGET_MPS) ? -1 : 0;

        if (targetSign == 0) {
            lastTargetSign = 0;
            lastEncoderMotionMs = nowMs;
            lastStallBoostMs = nowMs;
            stallBoostPwm = 0;
        } else {
            // A new direction starts a fresh breakaway attempt. Do not count
            // encoder movement from the previous direction as progress.
            if (targetSign != lastTargetSign) {
                lastTargetSign = targetSign;
                lastEncoderMotionMs = nowMs;
                lastStallBoostMs = nowMs;
                stallBoostPwm = 0;
            }

            if (delta != 0) {
                // Encoder movement is the feedback that the motor broke free.
                lastEncoderMotionMs = nowMs;
                lastStallBoostMs = nowMs;
                stallBoostPwm = 0;
            } else {
                unsigned long noMotionMs = nowMs - lastEncoderMotionMs;
                if (noMotionMs >= STALL_DETECT_MS &&
                    nowMs - lastStallBoostMs >= STALL_PWM_STEP_MS) {
                    stallBoostPwm = min(stallBoostPwm + STALL_PWM_STEP,
                                        STALL_PWM_MAX_BOOST);
                    lastStallBoostMs = nowMs;
                }
                if (noMotionMs >= STALL_ABORT_MS) {
                    stallFault = true;
                }
            }

            // Add boost only while the PID is asking in the target direction;
            // never fight the controller when it is braking or correcting overspeed.
            if (stallBoostPwm > 0 && out * targetMPS > 0.0f) {
                out += targetSign * stallBoostPwm;
            }
        }

        out = constrain(out, -255.0f, 255.0f);
        return out;
    }
};

PIDController pidLeft;
PIDController pidRight;

// ============================================================
// Command FSM
// ============================================================
enum MotionCommand { CMD_IDLE, CMD_FORWARD, CMD_TURN, CMD_VELOCITY };

MotionCommand currentCmd    = CMD_IDLE;
float         cmdTarget     = 0.0;  // เมตร (FORWARD) หรือ องศา (TURN)
float         rampedSpeed   = 0.0;
long          startLeftTicks  = 0;
long          startRightTicks = 0;

// Continuous Velocity Control (m/s) & Safety Watchdog
float         targetLeftSpeed   = 0.0f;
float         targetRightSpeed  = 0.0f;
unsigned long lastVelocityCmdTime = 0;
const unsigned long VELOCITY_TIMEOUT_MS = 300;  // ตัดมอเตอร์ทันทีหาก Serial ขาดหายเกิน 300ms

unsigned long lastControlTime  = 0;
unsigned long lastEncoderPrint = 0;
bool motionFaultLatched = false;
bool indicatorTestMode = false;
TurnSignal indicatorTestSignal = TURN_OFF;

// ============================================================
// Encoder ISR (PCINT1 — Port C)
// ผูกตามขา Pin ที่กำหนดไว้ใน robotconfig.h เป็นหลัก
// ============================================================
#define PORTC_BIT(pin)   ((pin) - A0)
#define PCINT_BIT(pin)   (PCINT8 + ((pin) - A0))

const uint8_t LEFT_A_BIT  = PORTC_BIT(LEFT_ENC_A);
const uint8_t LEFT_B_BIT  = PORTC_BIT(LEFT_ENC_B);
const uint8_t RIGHT_A_BIT = PORTC_BIT(RIGHT_ENC_A);
const uint8_t RIGHT_B_BIT = PORTC_BIT(RIGHT_ENC_B);

ISR(PCINT1_vect) {
    static uint8_t lastPortC = 0;
    uint8_t cur = PINC;

    // Left Encoder: Phase A rising edge
    if ((cur & (1 << LEFT_A_BIT)) && !(lastPortC & (1 << LEFT_A_BIT))) {
        if (cur & (1 << LEFT_B_BIT)) leftEncoderTicks--;
        else                         leftEncoderTicks++;
    }
    // Right Encoder: Phase A rising edge
    if ((cur & (1 << RIGHT_A_BIT)) && !(lastPortC & (1 << RIGHT_A_BIT))) {
        if (cur & (1 << RIGHT_B_BIT)) rightEncoderTicks--;
        else                          rightEncoderTicks++;
    }
    lastPortC = cur;
}

void setupEncoders() {
    pinMode(LEFT_ENC_A, INPUT_PULLUP);
    pinMode(LEFT_ENC_B, INPUT_PULLUP);
    pinMode(RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_B, INPUT_PULLUP);

    PCICR  |= (1 << PCIE1);  // เปิดใช้งาน Pin Change Interrupt บน Port C
    PCMSK1 |= (1 << PCINT_BIT(LEFT_ENC_A)) | (1 << PCINT_BIT(RIGHT_ENC_A));
}

// ============================================================
// Motor Driver
// ============================================================
void driveMotors(int leftPWM, int rightPWM) {
    int pwmL = constrain(abs(leftPWM), 0, 255);
    if      (leftPWM > 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
    else if (leftPWM < 0) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
    else                  { digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW); }
    analogWrite(ENA, pwmL);

    int pwmR = constrain(abs(rightPWM), 0, 255);
    if      (rightPWM > 0) { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
    else if (rightPWM < 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
    else                   { digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW); }
    analogWrite(ENB, pwmR);
}

// ============================================================
// Tick Reader (Thread-safe snapshot)
// ============================================================
void readTicks(long &leftOut, long &rightOut) {
    noInterrupts();
    long rawL = leftEncoderTicks;
    long rawR = rightEncoderTicks;
    interrupts();
    leftOut  = LEFT_ENC_INVERT  ? -rawL : rawL;
    rightOut = RIGHT_ENC_INVERT ? -rawR : rawR;
}

// ============================================================
// Start a new command: snapshot baseline ticks & reset PIDs
// ============================================================
void beginCommand(MotionCommand cmd, float target) {
    readTicks(startLeftTicks, startRightTicks);
    rampedSpeed = 0.0;
    pidLeft.reset();
    pidRight.reset();
    pidLeft.prevTicks = startLeftTicks;
    pidRight.prevTicks = startRightTicks;
    currentCmd = cmd;
    cmdTarget  = target;
}

void stopForEncoderStall() {
    if (motionFaultLatched) return;
    driveMotors(0, 0);
    currentCmd = CMD_IDLE;
    targetLeftSpeed = 0.0f;
    targetRightSpeed = 0.0f;
    motionFaultLatched = true;
    Serial.println("STATUS:STALL");
}

bool encoderStallDetected() {
    return pidLeft.stallFault || pidRight.stallFault;
}

// ============================================================
// Serial Command Parser
// ============================================================
void parseSerialCommand(const String &line) {
    if (TURN_INDICATOR_DEMO_MODE) {
        // Keep every drive output stopped until demo mode is disabled in config.
        driveMotors(0, 0);
        currentCmd = CMD_IDLE;
        targetLeftSpeed = 0.0f;
        targetRightSpeed = 0.0f;
        if (line == "STOP") {
            motionFaultLatched = false;
            Serial.println("STATUS:DONE");
        }
        return;
    }

    if (line.startsWith("LIGHTTEST:")) {
        String direction = line.substring(10);
        if (direction == "LEFT" || direction == "RIGHT") {
            driveMotors(0, 0);
            currentCmd = CMD_IDLE;
            targetLeftSpeed = 0.0f;
            targetRightSpeed = 0.0f;
            indicatorTestMode = true;
            indicatorTestSignal = (direction == "LEFT") ? TURN_LEFT : TURN_RIGHT;
            Serial.println("STATUS:LIGHTTEST");
        } else if (direction == "OFF") {
            driveMotors(0, 0);
            currentCmd = CMD_IDLE;
            targetLeftSpeed = 0.0f;
            targetRightSpeed = 0.0f;
            indicatorTestMode = false;
            indicatorTestSignal = TURN_OFF;
            Serial.println("STATUS:LIGHTTEST:OFF");
        } else {
            Serial.println("STATUS:ERROR");
        }
        return;
    }

    if (line == "STOP") {
        driveMotors(0, 0);
        currentCmd = CMD_IDLE;
        targetLeftSpeed = 0.0f;
        targetRightSpeed = 0.0f;
        motionFaultLatched = false;
        indicatorTestMode = false;
        indicatorTestSignal = TURN_OFF;
        pidLeft.reset();
        pidRight.reset();
        Serial.println("STATUS:DONE");
        return;
    }

    // In light-test mode, keep the wheels stopped and reject drive commands.
    if (indicatorTestMode) {
        driveMotors(0, 0);
        currentCmd = CMD_IDLE;
        targetLeftSpeed = 0.0f;
        targetRightSpeed = 0.0f;
        Serial.println("STATUS:LIGHTTEST");
        return;
    }

    if (motionFaultLatched && !line.startsWith("V:")) {
        Serial.println("STATUS:STALL");
        return;
    }

    if (line.startsWith("FORWARD:")) {
        float dist = line.substring(8).toFloat();
        if (dist > 0 && dist < 20.0) {
            beginCommand(CMD_FORWARD, dist);
        } else {
            Serial.println("STATUS:ERROR");
        }
        return;
    }

    if (line.startsWith("TURN:")) {
        float deg = line.substring(5).toFloat();
        beginCommand(CMD_TURN, deg);
        return;
    }

    // Continuous Velocity: V:<v_left>,<v_right> (e.g. V:0.250,0.250)
    if (line.startsWith("V:")) {
        int commaIdx = line.indexOf(',');
        if (commaIdx > 2) {
            float nextLeftSpeed  = line.substring(2, commaIdx).toFloat();
            float nextRightSpeed = line.substring(commaIdx + 1).toFloat();

            if (abs(nextLeftSpeed) < 0.005f && abs(nextRightSpeed) < 0.005f) {
                driveMotors(0, 0);
                targetLeftSpeed = 0.0f;
                targetRightSpeed = 0.0f;
                currentCmd = CMD_IDLE;
                motionFaultLatched = false;
                pidLeft.reset();
                pidRight.reset();
                return;
            }

            // Once a no-motion fault cuts drive power, ignore further nonzero
            // velocity updates until the host sends a zero command or STOP.
            if (motionFaultLatched) return;

            if (currentCmd != CMD_VELOCITY) {
                long leftTicks, rightTicks;
                readTicks(leftTicks, rightTicks);
                pidLeft.reset();
                pidRight.reset();
                pidLeft.prevTicks = leftTicks;
                pidRight.prevTicks = rightTicks;
            }
            targetLeftSpeed     = nextLeftSpeed;
            targetRightSpeed    = nextRightSpeed;
            currentCmd          = CMD_VELOCITY;
            lastVelocityCmdTime = millis();
            return;
        }
        Serial.println("STATUS:ERROR");
        return;
    }

    // Unknown command
    Serial.println("STATUS:ERROR");
}

// Map the commanded motion to the rear LED-matrix turn signal.
void updateTurnIndicator() {
    if (TURN_INDICATOR_DEMO_MODE) {
        static TurnSignal demoSignal = TURN_RIGHT;
        static unsigned long nextSideChange = 0;
        const unsigned long now = millis();
        const unsigned long sideDuration =
            TURN_SIGNAL_CYCLE_MS * TURN_INDICATOR_DEMO_CYCLES_PER_SIDE;

        if (nextSideChange == 0) {
            turnIndicatorSet(demoSignal);
            nextSideChange = now + sideDuration;
        } else if ((long)(now - nextSideChange) >= 0) {
            demoSignal = (demoSignal == TURN_RIGHT) ? TURN_LEFT : TURN_RIGHT;
            turnIndicatorSet(demoSignal);
            nextSideChange += sideDuration;
        }

        turnIndicatorUpdate();
        return;
    }

    TurnSignal signal = TURN_OFF;
    if (indicatorTestMode) {
        signal = indicatorTestSignal;
    } else if (currentCmd == CMD_TURN) {
        // TURN:+ is CCW (left); TURN:- is CW (right).
        signal = (cmdTarget >= 0.0f) ? TURN_LEFT : TURN_RIGHT;
    } else if (currentCmd == CMD_VELOCITY) {
        const float difference = targetRightSpeed - targetLeftSpeed;
        if (difference > 0.005f) signal = TURN_LEFT;
        else if (difference < -0.005f) signal = TURN_RIGHT;
    }
    turnIndicatorSet(signal);
    turnIndicatorUpdate();
}

// ============================================================
// Motion Execution (called at 50 Hz)
// ============================================================
void executeForward(float dt) {
    long leftTicks, rightTicks;
    readTicks(leftTicks, rightTicks);

    long dL = leftTicks  - startLeftTicks;
    long dR = rightTicks - startRightTicks;
    float distTraveled  = ((dL + dR) / 2.0f) * METERS_PER_PULSE;
    float distRemaining = cmdTarget - distTraveled;

    if (distRemaining <= 0.005f) {
        driveMotors(0, 0);
        currentCmd = CMD_IDLE;
        Serial.println("STATUS:DONE");
        return;
    }

    // Ramp target speed
    float desiredSpeed = CRUISE_SPEED_MPS;
    if (distRemaining < DECEL_DIST_METERS) {
        desiredSpeed = max((distRemaining / DECEL_DIST_METERS) * CRUISE_SPEED_MPS, MIN_DRIVE_SPEED);
    }

    if (rampedSpeed < desiredSpeed)
        rampedSpeed = min(rampedSpeed + ACCEL_STEP_MPS, desiredSpeed);
    else if (rampedSpeed > desiredSpeed)
        rampedSpeed = max(rampedSpeed - ACCEL_STEP_MPS, desiredSpeed);

    // Wheel Sync: คอมเพนเซท drift
    float posErr = (dL - dR) * METERS_PER_PULSE;
    float sync   = posErr * K_sync;

    float pwmL = pidLeft.compute(leftTicks,  rampedSpeed - sync, dt);
    float pwmR = pidRight.compute(rightTicks, rampedSpeed + sync, dt);

    if (encoderStallDetected()) {
        stopForEncoderStall();
        return;
    }

    driveMotors((int)pwmL, (int)pwmR);
}

void executeTurn(float dt) {
    long leftTicks, rightTicks;
    readTicks(leftTicks, rightTicks);

    // นับ ticks สะสมจากทั้งสองล้อ (Tank Turn)
    long dL   = abs(leftTicks  - startLeftTicks);
    long dR   = abs(rightTicks - startRightTicks);
    long done = (dL + dR) / 2;

    float targetTicks = abs(cmdTarget) * TICKS_PER_DEGREE;

    if (done >= (long)targetTicks) {
        driveMotors(0, 0);
        currentCmd = CMD_IDLE;
        Serial.println("STATUS:DONE");
        return;
    }

    // + องศา = CCW = ล้อซ้าย backward, ล้อขวา forward
    float dirSign = (cmdTarget >= 0) ? 1.0f : -1.0f;
    float pwmL = pidLeft.compute(leftTicks,  -TURN_SPEED_MPS * dirSign, dt);
    float pwmR = pidRight.compute(rightTicks, TURN_SPEED_MPS * dirSign, dt);

    if (encoderStallDetected()) {
        stopForEncoderStall();
        return;
    }

    driveMotors((int)pwmL, (int)pwmR);
}

void executeVelocity(float dt) {
    // Safety Watchdog: หากไม่ได้รับคำสั่ง V: ภายใน 300ms ให้หยุดมอเตอร์ทันที
    if (millis() - lastVelocityCmdTime > VELOCITY_TIMEOUT_MS) {
        targetLeftSpeed  = 0.0f;
        targetRightSpeed = 0.0f;
        driveMotors(0, 0);
        currentCmd = CMD_IDLE;
        return;
    }

    long leftTicks, rightTicks;
    readTicks(leftTicks, rightTicks);

    float pwmL = pidLeft.compute(leftTicks,  targetLeftSpeed, dt);
    float pwmR = pidRight.compute(rightTicks, targetRightSpeed, dt);

    if (encoderStallDetected()) {
        stopForEncoderStall();
        return;
    }

    driveMotors((int)pwmL, (int)pwmR);
}

// ============================================================
// setup / loop
// ============================================================
void setup() {
    Serial.begin(115200);
    turnIndicatorBegin();

    // กำหนด Pin ควบคุมมอเตอร์ L298 ตาม robotconfig.h
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    setupEncoders();

    pidLeft.Kp  = 150.0; pidLeft.Ki  = 10.0; pidLeft.Kd  = 1.2;
    pidRight.Kp = 150.0; pidRight.Ki = 10.0; pidRight.Kd = 1.2;

    Serial.println("STATUS:READY");
}

void loop() {
    updateTurnIndicator();
    unsigned long now = millis();

    // --- 50 Hz PID/Motion Loop ---
    if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
        float dt = (now - lastControlTime) / 1000.0f;
        lastControlTime = now;

        if (TURN_INDICATOR_DEMO_MODE) {
            currentCmd = CMD_IDLE;
            targetLeftSpeed = 0.0f;
            targetRightSpeed = 0.0f;
            driveMotors(0, 0);
        } else {
            switch (currentCmd) {
                case CMD_FORWARD:  executeForward(dt);  break;
                case CMD_TURN:     executeTurn(dt);     break;
                case CMD_VELOCITY: executeVelocity(dt); break;
                case CMD_IDLE:     driveMotors(0, 0);   break;
            }
        }
    }

    // --- Encoder Broadcast (100 ms) ---
    if (now - lastEncoderPrint >= ENCODER_PRINT_MS) {
        lastEncoderPrint = now;
        long L, R;
        readTicks(L, R);
        Serial.print("ENCODER:");
        Serial.print(L);
        Serial.print(",");
        Serial.println(R);
    }

    // --- Serial Command Receiver (Non-blocking) ---
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) {
            parseSerialCommand(line);
        }
    }
}
