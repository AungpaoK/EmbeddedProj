# Finite State Machine (FSM) - ระบบหุ่นยนต์ส่งอาหาร

## 1. Main Delivery FSM (ระบบจัดการการส่งอาหาร)

การทำงานจริงของ main.py รับรายการจาก POS ผ่าน HTTP ใน process เดียวกัน เว็บ
ส่งคำสั่งเข้าคิว thread-safe และ Main FSM เป็นผู้ควบคุมการเคลื่อนที่เอง
ผู้ใช้เลือกปลายทางให้ชั้นที่ใช้งานและยืนยันว่าของวางแล้ว จากนั้นยืนยันการรับ
อาหารที่แต่ละโต๊ะผ่านหน้าจอ หรือใช้ physical override เดิมแทนการยืนยันได้
รุ่นนี้ยังไม่ใช้ IR sensor สำหรับตรวจวางหรือหยิบอาหาร

```mermaid
stateDiagram-v2
    [*] --> WaitForPOS: start

    WaitForPOS: รอรายการจาก POS
    Delivering: ส่งอาหาร (เรียกใช้ Motion Sub-FSM)
    WaitForPickup: รอ POS ยืนยันรับอาหาร หรือ physical override
    CheckRemaining: มีอาหารที่ยังไม่ส่ง?
    ReturnStation: กลับ station (เรียกใช้ Motion Sub-FSM)
    Error: หยุดการเคลื่อนที่และรอการตรวจสอบ

    WaitForPOS --> Delivering: รับภารกิจที่ตรวจสอบแล้ว
    Delivering --> WaitForPickup: Motion Sub-FSM เสร็จสิ้น (ถึงโต๊ะ)
    Delivering --> Error: นำทางไม่สำเร็จ
    WaitForPickup --> CheckRemaining: ยืนยันรับอาหาร
    CheckRemaining --> Delivering: มี (ส่งโต๊ะถัดไป)
    CheckRemaining --> ReturnStation: ไม่มี
    ReturnStation --> WaitForPOS: Motion Sub-FSM เสร็จสิ้น (ถึง Station)
    ReturnStation --> Error: กลับ station ไม่สำเร็จ
```

รายการส่งเรียงตามหมายเลขชั้นจากน้อยไปมาก (ชั้น 1 ก่อนชั้น 2) เมื่อรอรับอาหาร
FSM จะไม่ข้ามรายการตามเวลา หากการนำทางล้มเหลว ระบบหยุดมอเตอร์และค้างสถานะ
Error จนกว่าจะตรวจสอบและกู้คืนหุ่นยนต์

POS มีเฉพาะใน main.py; scenario_runner.py ยังคงเป็น scenario runner แยก
และไม่ได้รับคำสั่งจากหน้า POS

---

## 2. Motion & LED Matrix Sub-FSM (ระบบควบคุมการเคลื่อนที่และไฟเลี้ยว)

เมื่อ Main FSM อยู่ในสถานะ `Delivering` หรือ `ReturnStation` ระบบจะส่งเป้าหมายพิกัด Waypoint $(x, y)$ ไปยัง **Motion Sub-FSM** เพื่อควบคุมมอเตอร์ร่วมกับ Encoder (คำนวณ Odometry $x, y, \theta$) และสั่งการแสดงผลบน **LED Matrix** แบบ Non-blocking พร้อมกัน

```mermaid
stateDiagram-v2
    [*] --> MOTION_IDLE

    state "MOTION_IDLE\n(หยุดนิ่ง / รอรับ Waypoint)\n[LED: Standby Mode]" as MOTION_IDLE
    state "MOTION_CALC_HEADING\n(คำนวณทิศทาง & มุมเลี้ยว)\n[LED: Standby Mode]" as MOTION_CALC_HEADING
    state "MOTION_TURN_LEFT\n(หมุนตัวเลี้ยวซ้ายด้วย PID)\n[LED: ไฟเลี้ยวซ้ายกะพริบ]" as MOTION_TURN_LEFT
    state "MOTION_TURN_RIGHT\n(หมุนตัวเลี้ยวขวาด้วย PID)\n[LED: ไฟเลี้ยวขวากะพริบ]" as MOTION_TURN_RIGHT
    state "MOTION_FORWARD\n(เคลื่อนที่ตรงด้วย Ramp Speed & PID Sync)\n[LED: ไฟท้าย / ลูกศรตรง]" as MOTION_FORWARD
    state "MOTION_BRAKE_ARRIVED\n(เบรกหยุด / ถึงจุดหมาย)\n[LED: ไฟเบรกเตือน]" as MOTION_BRAKE_ARRIVED

    MOTION_IDLE --> MOTION_CALC_HEADING: รับคำสั่ง Waypoint ใหม่
  
    MOTION_CALC_HEADING --> MOTION_TURN_LEFT: มุมเป้าหมายอยู่ทางซ้าย (Δθ > threshold)
    MOTION_CALC_HEADING --> MOTION_TURN_RIGHT: มุมเป้าหมายอยู่ทางขวา (Δθ < -threshold)
    MOTION_CALC_HEADING --> MOTION_FORWARD: ทิศทางตรงกับเป้าหมายแล้ว (|Δθ| ≤ threshold)

    MOTION_TURN_LEFT --> MOTION_FORWARD: หมุนได้มุมเป้าหมายแล้ว
    MOTION_TURN_RIGHT --> MOTION_FORWARD: หมุนได้มุมเป้าหมายแล้ว

    MOTION_FORWARD --> MOTION_BRAKE_ARRIVED: ระยะห่างถึง Waypoint ≤ ระยะหยุด (Distance Error ≤ tolerance)
    MOTION_FORWARD --> MOTION_CALC_HEADING: ยังไม่ถึง แต่ Heading เบี่ยงเบนเกินกำหนด

    MOTION_BRAKE_ARRIVED --> MOTION_IDLE: ความเร็วลดเหลือ 0 และแจ้ง Main FSM สำเร็จ
```

---

### รายละเอียดการทำงานของ Motion & LED Subsystem

#### A. การคำนวณตำแหน่งแบบ Odometry (Dead Reckoning)

ระบบอ่านค่าจาก Interrupt ของ Left/Right Encoder ทุก ๆ Control Loop (50 Hz):

- $\Delta d_{left} = \Delta \text{ticks}_{left} \times \text{METERS\_PER\_PULSE}$
- $\Delta d_{right} = \Delta \text{ticks}_{right} \times \text{METERS\_PER\_PULSE}$
- $\Delta d = \frac{\Delta d_{right} + \Delta d_{left}}{2}$
- $\Delta \theta = \frac{\Delta d_{right} - \Delta d_{left}}{\text{WHEEL\_BASE}}$
- อัปเดตพิกัด:

  $$
  x \leftarrow x + \Delta d \cdot \cos(\theta + \frac{\Delta \theta}{2})
  $$

  $$
  y \leftarrow y + \Delta d \cdot \sin(\theta + \frac{\Delta \theta}{2})
  $$

  $$
  \theta \leftarrow \theta + \Delta \theta
  $$

#### B. การควบคุมความเร็ว (Motion Profiling & Synchronization)

1. **Ramping (Accel/Cruise/Decel)**: ปรับอัตราเร่งขึ้นแบบนุ่มนวล และคำนวณจุด Deceleration Distance ล่วงหน้าเพื่อไม่ให้หัวทิ่มหรืออาหารหก
2. **PID & Wheel Sync**: ใช้ PID คุมความเร็วแต่ละล้อ พร้อม cross-coupling sync ($K_{sync}$) รักษาทิศทางตรง

#### C. การจัดการ LED Matrix (Non-blocking Engine via `millis()`)

- LED Matrix ไม่ใช้ฟังก์ชัน `delay()` เพื่อไม่รบกวน PID loop 50Hz
- อัปเดตแอนิเมชันผ่านตัวแปรจับเวลา `millis()` ตามสถานะของ Motion Sub-FSM:
  - **MOTION_TURN_LEFT**: รันแอนิเมชันลูกศรวิ่งชี้ไปทางซ้าย กะพริบทุก 200–250 ms
  - **MOTION_TURN_RIGHT**: รันแอนิเมชันลูกศรวิ่งชี้ไปทางขวา กะพริบทุก 200–250 ms
  - **MOTION_FORWARD**: ไฟแถบด้านท้ายวิ่ง หรือไฟสีปกติแสดงสถานะกำลังเดินหน้า
  - **MOTION_BRAKE_ARRIVED**: ไฟกระพริบสีแดงเตือนเบรก/จอด
  - **MOTION_IDLE**: แสดงไฟหรี่หรือโลโก้ Standby
