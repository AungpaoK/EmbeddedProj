# Food Delivery Robot Documentation

เอกสารการออกแบบระบบและสถาปัตยกรรมหุ่นยนต์ส่งอาหารอัตโนมัติ

## สารบัญเอกสาร (Table of Contents)

1. [คู่มือตั้งค่าระบบ (setup.md)](setup.md)
   - เตรียมอุปกรณ์ พอร์ต และ firmware
   - เปิด ROS 2, robot bridge, ผังร้าน, POS/Terminal console และตรวจสอบความพร้อม
   - รูปแบบข้อความ `KEY:` จาก Keypad และเส้นทางรับข้อมูลของ Pi

2. [แบบร่างสถาปัตยกรรมเดิม (design.md)](design.md)
   - เป็นแบบร่างช่วงแรกที่ยังอ้างถึง Arduino สองบอร์ดและ IR/LCD; การต่อใช้งานปัจจุบันให้ดู `setup.md`
   - โครงสร้างทางกายภาพ 3 ชั้น และระบบขับเคลื่อน 6 ล้อ (Differential Drive / Tank Turn)
   - การจัดสรรอุปกรณ์ตามชั้น (Control Base, Food Plates, LED Matrix, IR Sensors)
   - สถาปัตยกรรมการสื่อสารระหว่างบอร์ด (Raspberry Pi & Dual Arduino Uno R3)
   - ผังการเชื่อมต่อ Pinout (Motion Controller vs Shelf & UI Controller)
   - การวิเคราะห์ระบบไฟฟ้าและแหล่งจ่ายพลังงาน (Power Distribution & Isolation)

3. [บทที่ 2 แนวคิด ทฤษฎี และเอกสารที่เกี่ยวข้อง (chapter2.md)](chapter2.md)
   - ระบบฝังตัว การควบคุมมอเตอร์ Encoder และ PID
   - Differential Drive, Odometry, เซนเซอร์และความปลอดภัย
   - Serial, FSM, ระบบจ่ายไฟ และงานวิจัยที่เกี่ยวข้อง

4. [Finite State Machine Architecture (FSM.md)](FSM.md)
   - Main Delivery FSM (ระบบจัดการการส่งอาหาร)
   - Motion & LED Matrix Sub-FSM (ระบบควบคุมการเคลื่อนที่และไฟเลี้ยว)
   - การคำนวณตำแหน่งแบบ Odometry (Dead Reckoning)
   - การควบคุมความเร็ว Ramping & Non-blocking LED Matrix

5. [Operation Scenarios (scenario.md)](scenario.md)
   - แผนผังร้านอาหารและพิกัดเส้นทาง (Layout & Coordinates)
   - Scenario 1: การเสิร์ฟโต๊ะเดี่ยว (Single Table Delivery)
   - Scenario 2: การเสิร์ฟ 2 โต๊ะในรอบเดียว (Multi-Table Delivery)
   - Scenario 3: การใช้ปุ่ม Manual Override และจัดการข้อผิดพลาด
   - ตารางสอดประสาน State, Odometry, มอเตอร์ และไฟเลี้ยว LED Matrix
