#!/usr/bin/env python3
"""
generate_scenario_map.py — Generate Restaurant 2D Map for docs/scenario.md
========================================================================
สร้างไฟล์แผนที่ maps/restaurant_map.pgm และ maps/restaurant_map.yaml
ตรงตามขนาดและพิกัดที่ระบุใน docs/scenario.md:
  - Serve Station (Kitchen) ที่ (0, 0)
  - Junction ที่ (2.0, 0)
  - Table 1 ที่ (2.0, +0.6)
  - Table 2 ที่ (2.0, -0.6)
  - ทางเดินหลักกว้าง 0.8m พร้อมโต๊ะและผนังห้องอาหาร
"""

import os
from PIL import Image, ImageDraw

def generate_map():
    maps_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "maps")
    os.makedirs(maps_dir, exist_ok=True)
    
    pgm_path = os.path.join(maps_dir, "restaurant_map.pgm")
    yaml_path = os.path.join(maps_dir, "restaurant_map.yaml")

    # พารามิเตอร์ของแผนที่
    resolution = 0.05  # 5 cm ต่อ pixel
    
    # ขนาดพื้นที่จริง (เมตร): X [-1.2 ถึง +3.6], Y [-2.0 ถึง +2.0]
    # รวมความยาว X = 4.8m, ความกว้าง Y = 4.0m
    world_min_x = -1.2
    world_max_x = 3.6
    world_min_y = -2.0
    world_max_y = 2.0
    
    width_m = world_max_x - world_min_x   # 4.8 m
    height_m = world_max_y - world_min_y  # 4.0 m
    
    width_px = int(round(width_m / resolution))   # 96 pixels
    height_px = int(round(height_m / resolution)) # 80 pixels
    
    # ฟังก์ชันแปลงพิกัดโลก (m) -> พิกัดรูปภาพ (pixels)
    # หมายเหตุ: ใน ROS OccupancyGrid แถวล่างสุด (Y=0 ในภาพ) คือ Y ต่ำสุดในโลกจริง
    def to_pixel(x, y):
        px = int(round((x - world_min_x) / resolution))
        # สลับแกน Y เพราะภาพ PIL มี (0,0) อยู่มุมบนซ้าย แต่ ROS ถือว่า origin อยู่มุมล่างซ้าย
        py = int(round((world_max_y - y) / resolution))
        return px, py

    # ค่าสี Grayscale:
    # 254 = พื้นที่ว่างเดินได้ (Free space / White)
    # 0   = กำแพง / สิ่งกีดขวาง (Occupied / Black)
    # 205 = นอกอาคาร / ไม่ทราบค่า (Unknown / Light Gray)
    
    # เริ่มต้นด้วย Unknown (205)
    img = Image.new("L", (width_px, height_px), color=205)
    draw = ImageDraw.Draw(img)

    # 1. วาดกรอบห้องอาหารทั้งหมดเป็น Free Space (254)
    r_x1, r_y1 = to_pixel(-1.0, 1.8)
    r_x2, r_y2 = to_pixel(3.4, -1.8)
    draw.rectangle([min(r_x1, r_x2), min(r_y1, r_y2), max(r_x1, r_x2), max(r_y1, r_y2)], fill=254)

    # 2. วาดกำแพงรอบนอกห้องอาหาร (Black - 0) หนา ~3 pixels (15 cm)
    wall_x1, wall_y1 = to_pixel(-1.0, 1.8)
    wall_x2, wall_y2 = to_pixel(3.4, -1.8)
    draw.rectangle([min(wall_x1, wall_x2), min(wall_y1, wall_y2), max(wall_x1, wall_x2), max(wall_y1, wall_y2)], outline=0, width=3)

    # 3. กำแพงและเคาน์เตอร์ครัว (Serve Station / Kitchen)
    # ครัวอยู่โซน X: [-1.0, -0.4], Y: [-0.8, 0.8]
    # ปิดล้อมเหลือช่องประตูตรงกลางให้หุ่นยนต์วิ่งออกที่ (0, 0)
    k_w1, k_h1 = to_pixel(-0.4, 1.8)
    k_w2, k_h2 = to_pixel(-0.4, 0.5)
    draw.line([k_w1, k_h1, k_w2, k_h2], fill=0, width=3)

    k_w3, k_h3 = to_pixel(-0.4, -0.5)
    k_w4, k_h4 = to_pixel(-0.4, -1.8)
    draw.line([k_w3, k_h3, k_w4, k_h4], fill=0, width=3)

    # เคาน์เตอร์วางอาหารในครัว
    c_x1, c_y1 = to_pixel(-0.9, 0.4)
    c_x2, c_y2 = to_pixel(-0.5, -0.4)
    draw.rectangle([min(c_x1, c_x2), min(c_y1, c_y2), max(c_x1, c_x2), max(c_y1, c_y2)], fill=0)

    # 4. โต๊ะ 1 (Table 1)
    # พิกัดเป้าหมายจอดคือ (2.0, +0.6) โต๊ะตั้งอยู่ที่ Y: [+1.0 ถึง +1.6], X: [1.6 ถึง 2.4]
    t1_x1, t1_y1 = to_pixel(1.6, 1.6)
    t1_x2, t1_y2 = to_pixel(2.4, 1.0)
    draw.rectangle([min(t1_x1, t1_x2), min(t1_y1, t1_y2), max(t1_x1, t1_x2), max(t1_y1, t1_y2)], fill=0)

    # 5. โต๊ะ 2 (Table 2)
    # พิกัดเป้าหมายจอดคือ (2.0, -0.6) โต๊ะตั้งอยู่ที่ Y: [-1.6 ถึง -1.0], X: [1.6 ถึง 2.4]
    t2_x1, t2_y1 = to_pixel(1.6, -1.0)
    t2_x2, t2_y2 = to_pixel(2.4, -1.6)
    draw.rectangle([min(t2_x1, t2_x2), min(t2_y1, t2_y2), max(t2_x1, t2_x2), max(t2_y1, t2_y2)], fill=0)

    # 6. โต๊ะตกแต่งอื่นๆ เพิ่มเติมเพื่อให้ทางเดินเป็น Corridor สมจริง
    # โต๊ะเสริมด้านซ้ายบน X: [0.3, 1.1], Y: [1.0, 1.6]
    tb1_x1, tb1_y1 = to_pixel(0.3, 1.6)
    tb1_x2, tb1_y2 = to_pixel(1.1, 1.0)
    draw.rectangle([min(tb1_x1, tb1_x2), min(tb1_y1, tb1_y2), max(tb1_x1, tb1_x2), max(tb1_y1, tb1_y2)], fill=0)

    # โต๊ะเสริมด้านซ้ายล่าง X: [0.3, 1.1], Y: [-1.6, -1.0]
    tb2_x1, tb2_y1 = to_pixel(0.3, -1.0)
    tb2_x2, tb2_y2 = to_pixel(1.1, -1.6)
    draw.rectangle([min(tb2_x1, tb2_x2), min(tb2_y1, tb2_y2), max(tb2_x1, tb2_x2), max(tb2_y1, tb2_y2)], fill=0)

    # กำแพงกั้นทางตันสุดทางเดิน X: [3.0, 3.4], Y: [-0.6, 0.6]
    w_end1, h_end1 = to_pixel(3.0, 0.6)
    w_end2, h_end2 = to_pixel(3.0, -0.6)
    draw.line([w_end1, h_end1, w_end2, h_end2], fill=0, width=3)

    # บันทึกภาพ PGM (Grayscale)
    img.save(pgm_path)
    print(f"✓ สร้างไฟล์แผนที่สำเร็จ: {pgm_path} ({width_px}x{height_px} pixels)")

    # 7. เขียนไฟล์ metadata YAML
    yaml_content = f"""image: restaurant_map.pgm
mode: trinary
resolution: {resolution:.6f}
origin: [{world_min_x:.6f}, {world_min_y:.6f}, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
    with open(yaml_path, "w") as f:
        f.write(yaml_content)
    print(f"✓ สร้างไฟล์ Metadata สำเร็จ: {yaml_path}")
    print(f"  - ขนาดพื้นที่ : {width_m:.2f}m x {height_m:.2f}m")
    print(f"  - จุดกำเนิด   : ({world_min_x:.2f}, {world_min_y:.2f})")
    print(f"  - ความละเอียด : {resolution*100:.1f} cm/pixel")
    print("  - Waypoints:")
    print("      Serve Station : (0.0, 0.0)")
    print("      Junction      : (2.0, 0.0)")
    print("      Table 1       : (2.0, +0.6)")
    print("      Table 2       : (2.0, -0.6)")

if __name__ == "__main__":
    generate_map()
