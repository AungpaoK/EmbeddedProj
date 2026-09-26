# Finite State Machine (FSM) - ระบบหุ่นยนต์ส่งอาหาร

## 1. Delivery FSM

ภารกิจถูกสร้างใน POS Server บน Raspberry Pi ผ่านหน้าจอสัมผัส หรือผ่าน Keypad 4x4 ที่ต่อกับ Arduino Uno ตัวเดียวกัน สองช่องทางแก้ POS draft ชุดเดียวกัน; Keypad ไม่ได้สร้างภารกิจแยกจาก POS

~~~mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> PREPARING: POS หรือ Keypad ส่งรายการ
    PREPARING --> NAVIGATING: ตรวจรายการและเริ่มงานแล้ว
    NAVIGATING --> WAITING_PICKUP: ถึงโต๊ะ
    NAVIGATING --> ERROR: นำทางไม่สำเร็จ
    WAITING_PICKUP --> PICKUP_DELAY: POS หรือ Keypad ยืนยันรับอาหาร
    PICKUP_DELAY --> NAVIGATING: ยังมีโต๊ะถัดไป
    PICKUP_DELAY --> RETURNING: ส่งครบทุกโต๊ะ
    RETURNING --> COMPLETED: กลับถึงจุดเริ่ม
    RETURNING --> ERROR: กลับไม่สำเร็จ
    COMPLETED --> IDLE: พร้อมรับงานรอบใหม่
    ERROR --> IDLE: POS reset หรือกด * ที่ Keypad
~~~

ผู้ใช้ยืนยันว่ามีอาหารวางบนชั้นผ่าน POS/Keypad; ไม่มี IR sensor ตรวจการวางหรือหยิบอาหาร และไม่มี LCD แยกหรือปุ่ม Physical Override ในชุดฮาร์ดแวร์ปัจจุบัน การยืนยันรับอาหารที่โต๊ะทำผ่านหน้าจอ POS หรือกด <code>#</code> บน Keypad รายการส่งเรียงตามหมายเลขชั้นจากน้อยไปมาก

| สถานะ POS | ปุ่ม Keypad | ผล |
| --- | --- | --- |
| <code>IDLE</code> หรือ <code>COMPLETED</code> | <code>A</code> / <code>B</code> | เลือกชั้น 2 / ชั้น 1 และเปิดแผงเลือกบน POS |
| <code>IDLE</code> หรือ <code>COMPLETED</code> | <code>1</code> / <code>2</code> | กำหนดโต๊ะ 1 / โต๊ะ 2 ให้ชั้นที่เลือก |
| <code>IDLE</code> หรือ <code>COMPLETED</code> | <code>C</code> | สลับสถานะยืนยันอาหารของชั้นที่เลือก |
| <code>IDLE</code> หรือ <code>COMPLETED</code> | <code>D</code> / <code>*</code> | ล้างรายการชั้นที่เลือก / ล้างรายการทั้งหมด |
| <code>IDLE</code> หรือ <code>COMPLETED</code> | <code>#</code> | ส่งรายการที่พร้อมออกเป็นภารกิจ |
| <code>WAITING_PICKUP</code> | <code>#</code> | ยืนยันว่ารับอาหารแล้ว |
| <code>ERROR</code> | <code>*</code> | ส่งคำขอ reset ภารกิจ |
| สถานะอื่น | ปุ่มใด ๆ | ไม่เปลี่ยนภารกิจ |

หมายเหตุ: การกำหนดโต๊ะใหม่ใน POS Server ปัจจุบันตั้งสถานะอาหารเป็นยืนยันแล้ว และปุ่ม `C` เป็นการสลับสถานะ ให้ตรวจข้อมูลบน Kiosk ก่อนส่งภารกิจ

POS/Kiosk ยังคงเป็นจอแสดงรายการและสถานะเมื่อใช้ Keypad ควบคุม โดย Keypad ทำหน้าที่เป็นอินพุตทางเลือก ไม่ได้แทนจอแสดงผล

---

## 2. Motion Controller และ LED Matrix

Raspberry Pi คำนวณ waypoint และส่งคำสั่งเคลื่อนที่ผ่าน ROS 2/ <code>slam_bridge</code> ไปยัง Arduino ส่วน Arduino ทำงานควบคุมความเร็วของล้อและส่ง encoder feedback กลับให้ Pi สถานะคำสั่งใน firmware แบ่งตามคำสั่ง <code>FORWARD</code>, <code>TURN</code>, <code>V</code> และ <code>STOP</code>

~~~mermaid
stateDiagram-v2
    [*] --> MOTION_IDLE
    MOTION_IDLE --> MOTION_FORWARD: FORWARD distance
    MOTION_IDLE --> MOTION_TURN: TURN degrees
    MOTION_IDLE --> MOTION_VELOCITY: V left,right
    MOTION_FORWARD --> MOTION_IDLE: ถึงระยะเป้าหมาย
    MOTION_TURN --> MOTION_IDLE: ถึงมุมเป้าหมาย
    MOTION_VELOCITY --> MOTION_IDLE: ได้รับคำสั่งหยุดหรือหมดเวลา
    MOTION_FORWARD --> MOTION_FAULT: Encoder ไม่เคลื่อนที่ตามกำหนด
    MOTION_TURN --> MOTION_FAULT: Encoder ไม่เคลื่อนที่ตามกำหนด
    MOTION_VELOCITY --> MOTION_FAULT: Encoder ไม่เคลื่อนที่ตามกำหนด
    MOTION_FAULT --> MOTION_IDLE: STOP/คำสั่งศูนย์
~~~

คำสั่งความเร็วต่อเนื่องมี watchdog: หากไม่พบคำสั่ง <code>V:</code> ใหม่ภายใน 300 ms Arduino จะหยุด PWM มอเตอร์ การตรวจ encoder stall จะตัดกำลังเมื่อไม่พบการเคลื่อนที่ต่อเนื่องและส่ง <code>STATUS:STALL</code> กลับ Raspberry Pi

### Odometry แบบ Differential Drive

Raspberry Pi คำนวณ odometry จาก encoder ticks ซ้ายและขวา:

- <code>Δd_left = Δticks_left × METERS_PER_PULSE</code>
- <code>Δd_right = Δticks_right × METERS_PER_PULSE</code>
- <code>Δd = (Δd_right + Δd_left) / 2</code>
- <code>Δθ = (Δd_right − Δd_left) / WHEEL_BASE</code>

จากนั้นอัปเดตตำแหน่งด้วย

$$
x \leftarrow x + \Delta d \cos(\theta + \Delta\theta/2),\quad
y \leftarrow y + \Delta d \sin(\theta + \Delta\theta/2),\quad
\theta \leftarrow \theta + \Delta\theta
$$

Encoder ถูกอ่านใน control loop ของ Arduino ทุก 20 ms (50 Hz) และส่งค่า <code>ENCODER:&lt;L&gt;,&lt;R&gt;</code> ผ่าน Serial ทุก 100 ms

### PID และไฟเลี้ยว

Arduino ใช้ PID แยกสำหรับมอเตอร์ซ้าย/ขวาและ Wheel Sync เพื่อชดเชยความต่างของระยะล้อขณะเดินตรง การเคลื่อนที่แบบระยะทางใช้การเพิ่ม/ลดความเร็วแบบ ramp และลดความเร็วเมื่อใกล้เป้าหมาย

LED Matrix 32x8 ใช้แสดงลูกศรซ้ายหรือขวาตามคำสั่ง <code>INDICATOR:LEFT</code>, <code>INDICATOR:RIGHT</code> และ <code>INDICATOR:OFF</code> จาก Pi แอนิเมชันอัปเดตด้วย <code>millis()</code> โดยไม่หน่วง loop ควบคุมมอเตอร์ จอ Matrix รุ่นนี้แสดงไฟเลี้ยว ไม่ได้แสดงไฟเบรกหรือสถานะอาหาร
