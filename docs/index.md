# Food Delivery Robot Documentation

เอกสารการออกแบบระบบและสถาปัตยกรรมหุ่นยนต์ส่งอาหารอัตโนมัติ

## สารบัญเอกสาร (Table of Contents)

1. [Finite State Machine Architecture (FSM.md)](file:///home/jk/EmbeddedProj/docs/FSM.md)
   - Main Delivery FSM (ระบบจัดการการส่งอาหาร)
   - Motion & LED Matrix Sub-FSM (ระบบควบคุมการเคลื่อนที่และไฟเลี้ยว)
   - การคำนวณตำแหน่งแบบ Odometry (Dead Reckoning)
   - การควบคุมความเร็ว Ramping & Non-blocking LED Matrix

2. [Operation Scenarios (scenario.md)](file:///home/jk/EmbeddedProj/docs/scenario.md)
   - แผนผังร้านอาหารและพิกัดเส้นทาง (Layout & Coordinates)
   - Scenario 1: การเสิร์ฟโต๊ะเดี่ยว (Single Table Delivery)
   - Scenario 2: การเสิร์ฟ 2 โต๊ะในรอบเดียว (Multi-Table Delivery)
   - Scenario 3: การใช้ปุ่ม Manual Override และจัดการข้อผิดพลาด
   - ตารางสอดประสาน State, Odometry, มอเตอร์ และไฟเลี้ยว LED Matrix
