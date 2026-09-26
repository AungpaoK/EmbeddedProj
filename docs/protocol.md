# โปรโตคอล Raspberry Pi ↔ Arduino #1

เอกสารนี้อธิบายข้อความที่แลกเปลี่ยนระหว่าง Raspberry Pi กับ Arduino #1 (Motion Controller) ตามโค้ดปัจจุบัน ครอบคลุมคำสั่งควบคุมมอเตอร์ ข้อมูล Encoder และปุ่ม Keypad

## การเชื่อมต่อและรูปแบบข้อความ

Pi เชื่อมกับ Arduino ผ่าน USB Serial ที่ความเร็ว `115200` baud ข้อมูลเป็นข้อความบรรทัด ไม่ใช่แพ็กเก็ตไบนารี โดยแต่ละคำสั่งหรือข้อมูลมี prefix ระบุชนิด และจบด้วย newline (`\n`) ตัวอย่างเช่น `V:0.250,0.250\n` ฝั่ง Arduino อ่านจนถึง newline แล้วตัดช่องว่างหรืออักขระขึ้นบรรทัดใหม่ก่อนแยกคำสั่ง ส่วน `Serial.println()` ส่งบรรทัดพร้อมตัวจบบรรทัดกลับมายัง Pi

ข้อความจากหลายชนิดใช้ Serial stream เส้นเดียวกัน ข้อมูล Encoder และ Keypad จึงอาจปรากฏระหว่างรอผลคำสั่งเคลื่อนที่ได้ ให้แยกข้อความจาก prefix เช่น `ENCODER:`, `KEY:` และ `STATUS:` ไม่มี command ID หรือ checksum ในรูปแบบข้อความปัจจุบัน และคำสั่งความเร็ว `V:` ไม่มีข้อความตอบรับเฉพาะ

## คำสั่งจาก Pi ไป Arduino

| รูปแบบ payload | ตัวอย่าง | ความหมายและผลตอบกลับ |
| --- | --- | --- |
| `V:<v_left>,<v_right>` | `V:0.250,0.250` | กำหนดความเร็วล้อซ้ายและขวา หน่วย m/s ค่าติดลบหมายถึงทิศกลับ เครื่องส่งค่าซ้ำระหว่างควบคุมต่อเนื่อง; หากไม่มีคำสั่งใหม่นานกว่า 300 ms Arduino จะหยุดมอเตอร์ คำสั่ง `V:0.000,0.000` ใช้หยุดและเคลียร์ fault ที่ค้างอยู่ ไม่มี status ตอบรับ |
| `INDICATOR:<direction>` | `INDICATOR:LEFT` | ตั้งไฟเลี้ยว โดย `<direction>` เป็น `LEFT`, `RIGHT` หรือ `OFF` เมื่อรับค่าถูกต้อง Arduino ส่ง `STATUS:INDICATOR`; ค่าที่ไม่รู้จักส่ง `STATUS:ERROR` |
| `FORWARD:<distance_m>` | `FORWARD:1.250` | วิ่งตรงตามระยะ หน่วยเมตร ค่าที่ firmware ยอมรับต้องมากกว่า 0 และน้อยกว่า 20 เมื่อถึงระยะส่ง `STATUS:DONE`; ค่าไม่ถูกต้องส่ง `STATUS:ERROR` |
| `TURN:<degrees>` | `TURN:-90.0` | หมุนตามมุมหน่วยองศา ค่าบวกคือซ้าย/ทวนเข็มนาฬิกา (CCW); ค่าลบคือขวา/ตามเข็มนาฬิกา (CW) เมื่อทำเสร็จส่ง `STATUS:DONE` |
| `STOP` | `STOP` | หยุดมอเตอร์ทันที ยกเลิกคำสั่งที่กำลังทำ และเคลียร์ motion fault จากนั้นส่ง `STATUS:DONE` |

คำสั่งที่ส่งจากโค้ดเป็นข้อความ ASCII พร้อม newline โดย Pi จัดรูปแบบ `V:` และระยะ `FORWARD:` เป็นทศนิยม 3 ตำแหน่ง และ `TURN:` เป็นทศนิยม 1 ตำแหน่ง

### โหมดควบคุมการเคลื่อนที่

- **ROS (`MOTION_BACKEND=ros`)**: `slam_bridge.py` รับ `/cmd_vel` ภายใน Pi แปลงความเร็วเชิงเส้นและเชิงมุมเป็นความเร็วล้อซ้าย/ขวา แล้วส่ง `V:<v_left>,<v_right>` ไปยัง Arduino ส่วน `/turn_intent` และ `/delivery_mission_active` ใช้ควบคุมว่าจะส่ง `INDICATOR:` เมื่อใด `start_robot.sh` กำหนดให้ใช้โหมดนี้
- **Serial โดยตรง (`MOTION_BACKEND=serial`)**: `MotionClient` ส่ง `V:` ระหว่างการควบคุมความเร็วต่อเนื่อง และรองรับ `FORWARD:`, `TURN:` กับ `STOP` สำหรับคำสั่งแบบมีเป้าหมายหรือหยุดฉุกเฉิน รวมทั้งส่ง `INDICATOR:` เมื่อเปลี่ยนไฟเลี้ยว หากเรียก `main.py` โดยไม่กำหนด backend ค่าเริ่มต้นคือ `serial`

ชื่อ ROS topic เป็นช่องทางสื่อสารภายใน Pi ไม่ใช่ payload บนสาย USB Serial

## ข้อมูลจาก Arduino ไป Pi

| รูปแบบ payload | ตัวอย่าง | ความหมายและจังหวะส่ง |
| --- | --- | --- |
| `ENCODER:<left_ticks>,<right_ticks>` | `ENCODER:1234,-1228` | จำนวน Encoder ticks สะสมของล้อซ้ายและขวา เป็นจำนวนเต็มมีเครื่องหมาย ส่งทุก 100 ms ใช้คำนวณ odometry |
| `KEY:<char>` | `KEY:A` | ปุ่มใหม่จาก Keypad โดย `<char>` อยู่ในชุด `0–9`, `A–D`, `*`, `#` ส่งเมื่อมีการกดปุ่ม ไม่ใช่การส่งซ้ำทุก loop |
| `STATUS:READY` | `STATUS:READY` | ส่งเมื่อ Arduino เริ่มทำงานและพร้อม |
| `STATUS:DONE` | `STATUS:DONE` | แจ้งว่าคำสั่ง `FORWARD:`, `TURN:` หรือ `STOP` เสร็จแล้ว |
| `STATUS:INDICATOR` | `STATUS:INDICATOR` | แจ้งว่าได้รับคำสั่งตั้งไฟเลี้ยวที่ถูกต้อง |
| `STATUS:ERROR` | `STATUS:ERROR` | แจ้งคำสั่งที่ไม่รู้จัก หรือค่าคำสั่งที่ firmware ปฏิเสธ |
| `STATUS:STALL` | `STATUS:STALL` | แจ้งว่า encoder stall ทำให้ firmware ตัดการขับมอเตอร์; ต้องส่ง `STOP` หรือคำสั่งความเร็วศูนย์เพื่อเคลียร์ fault |

`ENCODER:` เป็น telemetry ที่ส่งเป็นระยะ ส่วน `KEY:` ส่งเมื่อมี event จากปุ่ม ข้อความ `STATUS:` ใช้รายงานการเริ่มทำงาน ผลคำสั่ง หรือ fault และอาจปะปนกับ telemetry ใน stream เดียวกัน

## Keypad และ I2C ภายใน Arduino

Keypad ของ Arduino #1 ต่อผ่าน PCF8574 บนบัส I2C ของ Arduino Uno (`SDA` ที่ A4, `SCL` ที่ A5; address เริ่มต้น `0x20`) เมื่อมีการกด Arduino จะแปลง event เป็น `KEY:<char>` แล้วส่งผ่าน USB Serial ให้ Pi ดังนั้น I2C เป็นการเชื่อมต่อระหว่าง Arduino กับ PCF8574 ไม่ใช่การเชื่อมต่อโดยตรงระหว่าง Pi กับ Arduino
