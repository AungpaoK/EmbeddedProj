# Food Delivery Robot Documentation

เอกสารการออกแบบระบบและสถาปัตยกรรมหุ่นยนต์ส่งอาหารอัตโนมัติ

## สารบัญเอกสาร (Table of Contents)

1. [คู่มือตั้งค่าระบบ (setup.md)](file:///home/jk/EmbeddedProj/docs/setup.md)
   - เตรียมอุปกรณ์ พอร์ต และ firmware
   - เปิด ROS 2, SLAM bridge, POS และตรวจสอบความพร้อม

2. [Hardware & System Architecture Design (design.md)](file:///home/jk/EmbeddedProj/docs/design.md)
   - โครงสร้างทางกายภาพ 3 ชั้น และระบบขับเคลื่อน 6 ล้อ (Differential Drive / Tank Turn)
   - การจัดสรรอุปกรณ์ตามชั้น (Control Base, Food Plates, LED Matrix, IR Sensors)
   - สถาปัตยกรรมการสื่อสารระหว่างบอร์ด (Raspberry Pi & Dual Arduino Uno R3)
   - ผังการเชื่อมต่อ Pinout (Motion Controller vs Shelf & UI Controller)
   - การวิเคราะห์ระบบไฟฟ้าและแหล่งจ่ายพลังงาน (Power Distribution & Isolation)

3. [Finite State Machine Architecture (FSM.md)](file:///home/jk/EmbeddedProj/docs/FSM.md)
   - Main Delivery FSM (ระบบจัดการการส่งอาหาร)
   - Motion & LED Matrix Sub-FSM (ระบบควบคุมการเคลื่อนที่และไฟเลี้ยว)
   - การคำนวณตำแหน่งแบบ Odometry (Dead Reckoning)
   - การควบคุมความเร็ว Ramping & Non-blocking LED Matrix

4. [Operation Scenarios (scenario.md)](file:///home/jk/EmbeddedProj/docs/scenario.md)
   - แผนผังร้านอาหารและพิกัดเส้นทาง (Layout & Coordinates)
   - Scenario 1: การเสิร์ฟโต๊ะเดี่ยว (Single Table Delivery)
   - Scenario 2: การเสิร์ฟ 2 โต๊ะในรอบเดียว (Multi-Table Delivery)
   - Scenario 3: การใช้ปุ่ม Manual Override และจัดการข้อผิดพลาด
   - ตารางสอดประสาน State, Odometry, มอเตอร์ และไฟเลี้ยว LED Matrix
