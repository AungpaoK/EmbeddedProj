#!/usr/bin/env bash
# ==============================================================================
# stop_slam.sh — สคริปต์สั่งปิดระบบ SLAM ทุกหน้าต่างทันที
# ==============================================================================

SESSION="slam"

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "กำลังปิดระบบ SLAM และหยุดคำสั่งทั้งหมด..."
    tmux kill-session -t "$SESSION"
    echo "✓ ปิด Session เรียบร้อยแล้ว"
else
    echo "ไม่มี SLAM Session ทำงานอยู่"
fi
