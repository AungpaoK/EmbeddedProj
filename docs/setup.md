# คู่มือตั้งค่าระบบหุ่นยนต์ส่งอาหาร

คู่มือนี้อธิบายการเริ่มระบบตามค่าใน `.env` ปัจจุบัน ซึ่งใช้ `MOTION_BACKEND=ros` โดย `slam_bridge` เป็นผู้เชื่อม ROS 2 กับ Arduino มอเตอร์ และ `main.py` เปิดหน้า POS พร้อมควบคุมภารกิจส่งอาหาร

## 1. เตรียมอุปกรณ์และซอฟต์แวร์

### อุปกรณ์

- Raspberry Pi ที่เชื่อมต่อกับ Arduino มอเตอร์ (Arduino #1) และ LiDAR
- Arduino #2 สำหรับเซนเซอร์ชั้นวางและปุ่ม Manual Override เป็นอุปกรณ์เสริม
- ก่อนทดลองสั่งวิ่ง ให้ยกล้อขับเคลื่อนพ้นพื้นหรือถอดไฟจากมอเตอร์ไดรเวอร์

### ซอฟต์แวร์

- ROS 2 Jazzy พร้อม environment setup ที่ `/opt/ros/jazzy/setup.bash`
- ROS workspace ที่ `~/ros2_ws/install/setup.bash` และแพ็กเกจ `sllidar_ros2` กับ `slam_toolbox`
- Python 3, pySerial และ `tmux`
- Arduino IDE CLI (`arduino`) เฉพาะกรณีต้องแฟลช firmware ใหม่
- Chromium/Chromium Browser หรือ Firefox สำหรับหน้า POS บนจอ Pi

หากรันผ่าน SSH และต้องการให้ POS เปิดบนจอ Pi ต้องมี desktop session ทำงานอยู่ (แนะนำเปิด auto-login desktop ด้วยผู้ใช้เดียวกับที่ SSH) สคริปต์จะเปิด kiosk ไปที่ `DISPLAY=:0` เป็นค่าเริ่มต้น และพยายามปลุกจอผ่าน `xset` หากติดตั้งไว้

## 2. ตรวจพอร์ตและตั้งค่า `.env`

เสียบอุปกรณ์แล้วดูชื่อพอร์ตแบบคงที่:

```bash
ls -l /dev/serial/by-id/
```

เพิ่มหรือแก้ค่าต่อไปนี้ในไฟล์ `.env` ที่รากโปรเจกต์ โดยแทนค่าตัวอย่างด้วยชื่อพอร์ตจริง:

```dotenv
MOTION_BACKEND=ros
MOTION_PORT=/dev/serial/by-id/<Arduino-มอเตอร์>
MOTION_SERIAL_BAUD=115200
LIDAR_PORT=/dev/serial/by-id/<LiDAR>
```

หากตั้ง `LIDAR_PORT` ไว้ สคริปต์จะใช้พอร์ตนั้น หากไม่ตั้ง ระบบจะค้นหาอุปกรณ์ LiDAR ให้อัตโนมัติเมื่อพบพอร์ตที่ตรงกันเพียงตัวเดียว

ถ้าผู้ใช้ Linux ยังไม่มีสิทธิ์ใช้พอร์ตอนุกรม ให้เพิ่มผู้ใช้ในกลุ่ม `dialout` แล้วออกจากระบบและเข้าใหม่:

```bash
sudo usermod -aG dialout "$USER"
```

พิกัดเริ่มต้นของระบบกำหนดครัวเป็น `(0, 0)` หันหน้าไปทาง `+X`, ทางแยกห่าง `2.0 m`, โต๊ะ 1 อยู่ `y=+0.6 m` และโต๊ะ 2 อยู่ `y=-0.6 m` ปรับ `JUNCTION_X`, `TABLE1_Y` และ `TABLE2_Y` ใน `.env` ให้ตรงกับพื้นที่จริงก่อนใช้งาน [ค่าพิกัดและพอร์ต](/home/jk/EmbeddedProj/src/config.py:27)

## 3. เตรียม firmware ของ Arduino

Arduino #1 ต้องใช้ firmware ใน `src/Arduino_1_Motion/Arduino_1_Motion.ino` ถ้ายังไม่ได้แฟลช ให้หยุด SLAM bridge ก่อน แล้วรัน:

```bash
cd /home/jk/EmbeddedProj
bash flash_motion.sh
```

สคริปต์ตรวจพอร์ตและคอมไพล์ก่อนอัปโหลด จากนั้นให้พิมพ์ `FLASH` เพื่อยืนยัน ตรวจสอบว่าพอร์ตที่เลือกเป็น Arduino มอเตอร์ และยกล้อพ้นพื้นหรือถอดไฟมอเตอร์ไดรเวอร์ในการทดสอบครั้งแรก [สคริปต์แฟลช firmware](/home/jk/EmbeddedProj/flash_motion.sh)

Arduino #2 ไม่จำเป็นสำหรับเริ่มใช้ POS ค่าตั้งต้น `SHELF_SERIAL_PORT` คือ `none` จึงเลือก `VirtualShelfClient` แทน หากต้องการใช้บอร์ดจริง ให้กำหนดพอร์ต เช่น:

```dotenv
SHELF_PORT=/dev/serial/by-id/<Arduino-ชั้นวาง>
```

firmware ปัจจุบันของ Arduino #2 กำหนดปุ่ม Override ไว้ที่ D13; ตรวจ wiring ให้ตรงกับ [Arduino_2_Shelf.ino](/home/jk/EmbeddedProj/src/Arduino_2_Shelf/Arduino_2_Shelf.ino:34) ก่อนต่อใช้งาน

## 4. เปิดระบบทั้งหมดด้วยสคริปต์เดียว

สคริปต์รวมจะเปิด LiDAR, `slam_bridge`, ผังร้านและเส้นทางสำหรับ RViz, POS controller และ POS kiosk ใน tmux session ชื่อ `food-robot` โดยไม่ต้องค้าง SSH ไว้ ระบบใช้งานจริงเดินตามเส้นทางปิดรอบ Kitchen → Junction → Table เหมือน `start_scenario.sh`; ผังร้านเป็น visualization และไม่ได้ใช้ Nav2 วางแผนอ้อมสิ่งกีดขวาง

```bash
cd ~/EmbeddedProj
bash start_robot.sh
```

เมื่อเปิดสำเร็จ คำสั่งจะคืน prompt กลับมา แต่บริการทั้งหมดยังทำงานใน tmux หากเคยเปิด stack รุ่นเดิมด้วย `start_slam.sh` หรือเปิด POS ใน tmux ชื่อ `pos` ให้หยุด session เดิมก่อน เพื่อไม่ให้เปิด serial port ซ้ำ

ดู log ทุกโปรเซสหรือกลับเข้า session:

```bash
bash start_robot.sh status
bash start_robot.sh attach
```

ขณะอยู่ใน tmux กด `Ctrl+B` แล้ว `D` เพื่อ detach โดยไม่หยุดระบบ

POS kiosk จะเปิดบนจอ Pi ที่ `DISPLAY=:0` โดยอัตโนมัติ สคริปต์ตั้ง `XDG_RUNTIME_DIR`, หา Xauthority ของผู้ใช้ และพยายามเปิดจอจากโหมดพัก หาก desktop session ยังไม่ทำงาน หรือ SSH ด้วยคนละผู้ใช้กับ desktop ให้ตั้ง `POS_DISPLAY`, `POS_XAUTHORITY`, `POS_XDG_RUNTIME_DIR` หรือ `POS_WAYLAND_DISPLAY` ก่อนเรียกสคริปต์ เช่น:

```bash
POS_DISPLAY=:0 bash start_robot.sh
```

Firefox kiosk ใช้โปรไฟล์ชั่วคราวใหม่ทุกครั้ง แล้วลบเมื่อปิด browser เพื่อไม่ชนกับ lock ของรอบก่อน ตรวจ log การเปิดหน้าจอได้ที่ `/tmp/pos_kiosk_autostart.log`

หากต้องการควบคุมจาก terminal แทนหน้าจอสัมผัส ให้เปิด console หลังจาก stack ทำงานแล้ว:

```bash
bash start_robot.sh console
```

Console และ POS ใช้ API และสถานะภารกิจชุดเดียวกัน จึงสร้างงานพร้อมกันไม่ได้ แต่สามารถใช้ช่องทางใดช่องทางหนึ่งเพื่อสร้างงานหรือยืนยันการรับอาหารได้ เปิด RViz จากคอมด้วย:

```bash
./run_rviz2_pc.sh src/scenario_view.rviz
```

## 5. เปิดหน้า POS จากคอมผ่าน SSH tunnel

ตัว POS รับเฉพาะการเชื่อมต่อจากเครื่อง Pi (`127.0.0.1`) ถ้าต้องการดูหรือกด POS จากคอม ให้เปิด terminal บนคอมแล้วคง SSH tunnel นี้ไว้:

```bash
ssh -N -L 8765:127.0.0.1:8765 <user>@<IP-ของ-Pi>
```

จากนั้นเปิด `http://127.0.0.1:8765/` ใน browser บนคอม หน้าจอ POS บน Pi จะเปิดแยกใน kiosk ด้วย

## 6. ตรวจสอบก่อนเริ่มงาน

ใน terminal ที่ source ROS environment แล้ว ตรวจหัวข้อ ROS และ health endpoint:

```bash
ros2 topic echo /arduino/ready --once
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo /map --once
curl http://127.0.0.1:8765/api/health
```

`/arduino/ready` ควรแสดง `data: true`, `/scan` และ `/odom` ควรมีข้อมูลต่อเนื่อง และ health endpoint ควรตอบ `{"ok": true}` จากนั้นทดลองเคลื่อนที่โดยไม่มีอาหารก่อน และตรวจว่าพิกัดทางแยกกับโต๊ะตรงกับพื้นที่จริง

ตัวควบคุม LiDAR หยุดรถเมื่อพบวัตถุในกรวยด้านหน้าและเดินต่อเมื่อทางโล่ง แต่ไม่ได้วางแผนอ้อมสิ่งกีดขวาง ระบบส่งอาหารให้ยืนยันรับอาหารผ่าน POS, Terminal console หรือปุ่ม physical override; เซนเซอร์ IR ไม่ได้ใช้ตัดสินการรับอาหาร

## 7. ปิดระบบ

สั่งหยุดอย่างสุภาพจาก SSH:

```bash
bash ~/EmbeddedProj/start_robot.sh stop
```

## หมายเหตุ

`start_robot.sh` ต้องใช้ `MOTION_BACKEND=ros` เพราะ `slam_bridge` เป็นผู้เปิด serial ของ Arduino มอเตอร์ ห้ามรันตัวควบคุมแบบ serial หรือ teleop พร้อมภารกิจส่งอาหาร

หาก kiosk ไม่ปรากฏบนจอ ให้ยืนยันก่อนว่า Pi เข้าสู่ desktop session แล้ว และใช้ SSH ด้วยบัญชีเดียวกับ desktop จากนั้นตรวจ `/tmp/pos_kiosk_autostart.log` และค่า `POS_DISPLAY`/`POS_XAUTHORITY`

หาก Firefox แจ้งว่าเปิดอยู่แล้วแต่ไม่ตอบสนอง ให้ปิดหน้าต่างหรือโปรเซส Firefox เก่าบน Pi หนึ่งครั้ง แล้วเริ่ม kiosk ใหม่ ตัวเปิดปัจจุบันสร้างโปรไฟล์ชั่วคราวแยกในแต่ละครั้ง เพื่อลดปัญหา lock จากรอบก่อน

หาก Firefox kiosk เปิดเป็นหน้าดำบน desktop ที่ใช้ Wayland ให้ลองบังคับ Firefox Snap ผ่าน XWayland โดยเพิ่มค่านี้ใน `.env`:

```dotenv
MOZ_ENABLE_WAYLAND=0
DISABLE_WAYLAND=1
```

launcher จะอ่านพาธ Xauthority ปัจจุบันจากโปรเซส XWayland เพื่อรองรับ cookie ที่เปลี่ยนเมื่อ desktop session เริ่มใหม่ โดยปกติไม่ต้องกำหนด `POS_XAUTHORITY` เอง หากเคยใส่พาธ `.mutter-Xwaylandauth.*` แบบเจาะจงไว้ใน `.env` ให้ลบบรรทัดนั้นหลังอัปเดต launcher แล้ว หากยังเปิดไม่ได้ ให้ตรวจบรรทัด `Authorization required` ใน log และยืนยันว่า `POS_DISPLAY` ตรงกับ display ที่ XWayland ใช้ จากนั้นหยุดและเริ่ม stack ใหม่

หาก touchscreen ใช้งานได้ช่วงสั้น ๆ แล้วหยุด ให้ต่อสาย USB สำหรับข้อมูล touch เข้ากับ Pi และจ่ายไฟให้จอผ่านแหล่งจ่ายไฟของจอแยก จากการทดสอบ อุปกรณ์ touch มีการ disconnect/reconnect ขณะใช้ไฟจาก USB และทำงานต่อเนื่องเมื่อจอมีไฟเลี้ยงแยก อาการนี้ชี้ไปที่ความไม่เสถียรของไฟเลี้ยง USB มากกว่าการแย่ง bandwidth ระหว่างข้อมูล LiDAR, Arduino และ touch
