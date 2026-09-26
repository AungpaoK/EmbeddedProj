# Food Delivery Robot Documentation

เอกสารการออกแบบระบบและสถาปัตยกรรมหุ่นยนต์ส่งอาหารอัตโนมัติรุ่นปัจจุบัน

## สารบัญเอกสาร

1. [คู่มือตั้งค่าระบบ (setup.md)](setup.md)
   - เตรียม Raspberry Pi, Arduino Uno, LiDAR, POS/Kiosk และ Keypad
   - แฟลช firmware, ตั้งค่าพอร์ต และเริ่ม ROS 2 stack
   - โปรโตคอล Keypad ผ่าน USB Serial และ <code>/keypad/key</code>

2. [Hardware & System Architecture Design (design.md)](design.md)
   - โครงสร้างรถ 3 ชั้นและ Differential Drive 6 ล้อ
   - BOM ล่าสุดและการจ่ายไฟจากแบตเตอรี่ 12 V
   - Arduino Uno ตัวเดียว, PCF8574/I2C, Encoder, L298N และ LED Matrix
   - การเชื่อมต่อ Raspberry Pi, RPLIDAR และ POS/Kiosk

3. [บทที่ 2 แนวคิด ทฤษฎี และเอกสารที่เกี่ยวข้อง (chapter2.md)](chapter2.md)
   - ระบบฝังตัว, การควบคุมมอเตอร์, Encoder, PID และ Odometry
   - Keypad ผ่าน I2C, Serial, LiDAR safety guard และ FSM
   - ระบบจ่ายไฟและงานวิจัยที่เกี่ยวข้อง

4. [Finite State Machine Architecture (FSM.md)](FSM.md)
   - Delivery FSM และการควบคุมงานจาก POS/Keypad
   - คำสั่ง Motion Controller และการแสดงไฟเลี้ยว
   - Odometry, PID, Wheel Sync และ fault handling

5. [โปรโตคอลสื่อสารระหว่าง Raspberry Pi กับ Arduino (protocol.md)](protocol.md)
   - USB Serial และรูปแบบ payload ของคำสั่งกับข้อมูลตอบกลับ
   - โหมด ROS/Serial, Encoder telemetry และ Keypad

6. [หลักการควบคุมการเดินของหุ่นยนต์ (motion.md)](motion.md)
   - เส้นทางคำสั่งจาก POS/FSM ไปยัง Raspberry Pi และ Arduino Uno
   - คณิตศาสตร์ differential drive, encoder odometry และ heading correction
   - วงควบคุมความเร็วล้อ, safety pause และค่าที่ใช้จูน

7. [Operation Scenarios (scenario.md)](scenario.md)
   - พิกัดร้านและเส้นทาง Kitchen → Junction → Table
   - งานส่งโต๊ะเดี่ยวและหลายโต๊ะ
   - การยืนยันรับอาหาร, การหยุดเมื่อพบสิ่งกีดขวาง และการกู้คืน Error
