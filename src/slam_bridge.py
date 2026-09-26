#!/usr/bin/env python3
"""
slam_bridge.py — Standalone SLAM & Teleop Bridge for ROS 2
===========================================================
เครื่องมือสำหรับทดสอบ SLAM โดยเฉพาะ (ตัดเรื่องระบบส่งอาหารออกไปก่อน)

หน้าที่:
  1. บริดจ์คำสั่งความเร็ว /cmd_vel (Twist) จากคีย์บอร์ด -> ส่ง V:v_L,v_R ให้ Arduino #1
  2. อ่าน Encoder จาก Arduino #1 -> Publish /odom และบรอดคาสต์ TF: odom -> base_link
  3. บรอดคาสต์ Static TF: base_link -> laser_frame (พร้อมชดเชย Yaw Offset)
  4. รับ KEY:<char> จาก Arduino แล้ว Publish /keypad/key ให้ระบบ POS
  5. หากยังไม่ได้ต่อ Arduino #1 จะเปิด Dummy TF ให้ทดสอบสแกนด้วย LiDAR เพียวๆ ได้
"""

import os
import sys
import math
import time
import threading
import logging
import copy

try:
    import serial
except ImportError:
    serial = None

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist, TransformStamped
    from nav_msgs.msg import Odometry as OdomMsg
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String
    import tf2_ros
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

from config import (
    WHEEL_BASE,
    WHEEL_RADIUS,
    TICKS_PER_REV,
    METERS_PER_TICK,
    MOTION_SERIAL_PORT,
    MOTION_SERIAL_BAUD,
    SERIAL_TIMEOUT,
)
from arduino_serial import reset_and_wait_for_encoder
from turn_indicator import TURN_OFF, display_signal_for_intent

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("SLAMBridge")

ARDUINO_DTR_ATTEMPTS = 2
ARDUINO_DTR_PULSE_SECONDS = 0.25
ARDUINO_ENCODER_WAIT_SECONDS = 10.0
ARDUINO_STARTUP_READ_TIMEOUT = 0.1


class SlamBridgeNode(Node):
    def __init__(self, ser=None, yaw_offset_deg: float = 0.0):
        super().__init__("slam_bridge_node")
        self._ser = ser
        self._delivery_mission_active = False
        self._displayed_turn_signal = TURN_OFF
        self._yaw_offset = math.radians(yaw_offset_deg)
        self._laser_x = float(os.environ.get("LIDAR_OFFSET_X", "0.15"))
        self._laser_y = float(os.environ.get("LIDAR_OFFSET_Y", "0.0"))
        self._self_filter_radius = float(os.environ.get("SELF_FILTER_RADIUS", "0.195"))
        self._last_self_filter_log = 0.0

        # Odometry State
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._prev_l = 0
        self._prev_r = 0
        self._first_enc = True
        self._last_encoder_time = 0.0
        self._arduino_ready = False
        self._last_not_ready_cmd_log = 0.0
        self._last_not_ready_status_log = 0.0

        # TF Broadcasters
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Publishers & Subscribers
        self._odom_pub = self.create_publisher(OdomMsg, "/odom", 10)
        self._arduino_ready_pub = self.create_publisher(Bool, "/arduino/ready", 10)
        self._keypad_pub = self.create_publisher(String, "/keypad/key", 10)
        self._cmd_sub = self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_callback, 10)
        self._delivery_active_sub = self.create_subscription(
            Bool, "/delivery_mission_active", self._delivery_mission_callback, 10
        )
        self._turn_intent_sub = self.create_subscription(
            String, "/turn_intent", self._turn_intent_callback, 10
        )
        # Reassert the selected indicator so it resumes if the Arduino resets
        # while the motors are drawing current. This never publishes motor commands.
        self._indicator_heartbeat_timer = self.create_timer(
            0.5, self._reassert_turn_signal
        )
        self._scan_pub = self.create_publisher(LaserScan, "/scan_filtered", qos_profile_sensor_data)
        self._scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self._filter_self_scan,
            qos_profile_sensor_data,
        )

        logger.info(
            "LiDAR self-filter: circular footprint radius=%.3fm, "
            "laser offset=(%.3f, %.3f)m; publishing /scan_filtered",
            self._self_filter_radius,
            self._laser_x,
            self._laser_y,
        )

        # Direction inversion settings (แก้ปัญหามอเตอร์กลับขั้ว / เดินถอยหลัง / เลี้ยวกลับด้าน)
        self._invert_linear = os.environ.get("INVERT_LINEAR", "1") == "1"
        self._invert_steer = os.environ.get("INVERT_STEER", "1") == "1"
        # ROS uses positive yaw for counter-clockwise rotation. The encoder
        # differential already follows that convention on this robot.
        self._invert_odom_yaw = os.environ.get("INVERT_ODOM_YAW", "0") == "1"
        self._invert_left_enc = os.environ.get("INVERT_LEFT_ENC", "0") == "1"
        self._invert_right_enc = os.environ.get("INVERT_RIGHT_ENC", "0") == "1"

        # Differential-drive odometry track-width calibration only.
        # A physical 360-degree in-place turn measured about 426.7 degrees at
        # the nominal 0.343 m track, so widen the odometry track by 18.5%.
        # Keep WHEEL_BASE unchanged for converting cmd_vel into wheel speeds.
        track_factor = os.environ.get(
            "ODOM_TRACK_WIDTH_FACTOR",
            os.environ.get("SKID_FACTOR", "1.185"),
        )
        self._odom_track_factor = float(track_factor)
        self._effective_track_width = WHEEL_BASE * self._odom_track_factor

        logger.info(
            f"Drive Config: InvertLinear={self._invert_linear}, InvertSteer={self._invert_steer}, "
            f"InvertOdomYaw={self._invert_odom_yaw}, LeftEncInv={self._invert_left_enc}, "
            f"RightEncInv={self._invert_right_enc}, OdomTrackFactor={self._odom_track_factor} "
            f"(EffectiveTrack={self._effective_track_width:.3f}m)"
        )
        logger.info("Turn indicator is gated by the active food-delivery mission state.")

        # Broadcast Static TF: base_link -> laser และ laser_frame (ทิศทางของ LiDAR)
        self._broadcast_static_laser_tf()

        # Timer สำหรับ Publish Odometry และ TF ที่ 20 Hz
        self._timer = self.create_timer(0.05, self._publish_odom_and_tf)
        self._ready_timer = self.create_timer(0.2, self._publish_arduino_ready)

        # Serial Reader Thread (ถ้าต่อ Arduino)
        self._running = True
        if self._ser:
            self._reader_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
            self._reader_thread.start()
            logger.info(
                "Arduino serial port is open; motion stays disabled until valid ENCODER frames are fresh."
            )
        else:
            logger.warning("No Arduino connected. Running with Pure Laser / Static Odom mode.")

    def _publish_arduino_ready(self):
        """Report healthy Arduino communication only while valid encoder frames are fresh."""
        now = time.monotonic()
        ready = bool(
            self._ser
            and self._last_encoder_time > 0.0
            and now - self._last_encoder_time <= 1.0
        )
        if ready != self._arduino_ready:
            if ready:
                logger.info("Arduino ready: valid ENCODER stream received.")
            else:
                logger.error("Arduino not ready: valid ENCODER stream missing or stale.")
            self._arduino_ready = ready
        if self._ser and not ready and now - self._last_not_ready_status_log >= 5.0:
            logger.warning(
                "No valid ENCODER frames from %s at configured %d baud; "
                "refusing odometry and non-zero drive commands.",
                getattr(self._ser, "port", "unknown port"),
                MOTION_SERIAL_BAUD,
            )
            self._last_not_ready_status_log = now

        status = Bool()
        status.data = ready
        self._arduino_ready_pub.publish(status)

    def _broadcast_static_laser_tf(self):
        half_yaw = self._yaw_offset / 2.0
        transforms = []
        for child in ["laser", "laser_frame"]:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "base_link"
            t.child_frame_id = child
            t.transform.translation.x = self._laser_x
            t.transform.translation.y = self._laser_y
            t.transform.translation.z = 0.10
            t.transform.rotation.z = math.sin(half_yaw)
            t.transform.rotation.w = math.cos(half_yaw)
            transforms.append(t)
        for t in transforms:
            self._static_tf_broadcaster.sendTransform(t)

    def _filter_self_scan(self, scan: LaserScan):
        """Remove LiDAR returns that land inside the robot's circular footprint."""
        filtered = copy.deepcopy(scan)
        radius_sq = self._self_filter_radius * self._self_filter_radius
        masked_count = 0

        for i, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or distance < scan.range_min or distance > scan.range_max:
                continue

            angle = scan.angle_min + i * scan.angle_increment + self._yaw_offset
            x = self._laser_x + distance * math.cos(angle)
            y = self._laser_y + distance * math.sin(angle)
            if x * x + y * y <= radius_sq:
                # NaN marks a missing return, so SLAM neither inserts the robot
                # as an obstacle nor ray-traces through the robot to the room.
                filtered.ranges[i] = float("nan")
                masked_count += 1

        self._scan_pub.publish(filtered)
        now = time.monotonic()
        if masked_count and now - self._last_self_filter_log >= 5.0:
            logger.info("LiDAR self-filter removed %d/%d robot returns", masked_count, len(scan.ranges))
            self._last_self_filter_log = now

    def _cmd_vel_callback(self, msg: Twist):
        """รับความเร็วจาก teleop แล้วส่ง V:left,right ให้ Arduino"""
        if not self._ser:
            return

        if not self._arduino_ready and (abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001):
            now = time.monotonic()
            if now - self._last_not_ready_cmd_log >= 2.0:
                logger.error("Ignoring non-zero /cmd_vel: Arduino encoder stream is not healthy.")
                self._last_not_ready_cmd_log = now
            return

        # สลับทิศทางหากตั้งค่า Invert ไว้ (เช่น กด i แล้วถอยหลัง / กด j แล้วเลี้ยวขวา)
        v = -msg.linear.x if self._invert_linear else msg.linear.x
        w = -msg.angular.z if self._invert_steer else msg.angular.z

        v_l = v - (w * WHEEL_BASE / 2.0)
        v_r = v + (w * WHEEL_BASE / 2.0)

        try:
            self._ser.write(f"V:{v_l:.3f},{v_r:.3f}\n".encode("utf-8"))
            self._ser.flush()
        except Exception as e:
            logger.error(f"Serial write error: {e}")

    def _delivery_mission_callback(self, msg: Bool):
        active = bool(msg.data)
        self._delivery_mission_active = active
        self._send_turn_signal(TURN_OFF)

    def _turn_intent_callback(self, msg: String):
        if not self._delivery_mission_active:
            self._send_turn_signal(TURN_OFF)
            return
        try:
            signal = display_signal_for_intent(
                msg.data,
                invert_steer=self._invert_steer,
            )
        except ValueError as exc:
            logger.warning("Ignoring invalid turn intent %r: %s", msg.data, exc)
            return
        self._send_turn_signal(signal)

    def _reassert_turn_signal(self):
        desired = self._displayed_turn_signal if self._delivery_mission_active else TURN_OFF
        if desired != TURN_OFF or self._displayed_turn_signal != TURN_OFF:
            self._send_turn_signal(desired, force=True)

    def _send_turn_signal(self, signal: str, *, force: bool = False):
        if not self._ser or (signal == self._displayed_turn_signal and not force):
            return
        try:
            self._ser.write(f"INDICATOR:{signal}\n".encode("utf-8"))
            self._ser.flush()
            self._displayed_turn_signal = signal
        except Exception as exc:
            logger.error("Turn indicator serial write failed: %s", exc)

    def _serial_read_loop(self):
        """อ่านค่า ENCODER:<L>,<R> จาก Arduino #1"""
        last_rx_time = time.monotonic()
        last_no_data_warning = 0.0
        last_read_error_log = 0.0
        encoder_seen = False

        while self._running:
            try:
                if self._ser.in_waiting == 0:
                    now = time.monotonic()
                    if now - last_rx_time >= 3.0 and now - last_no_data_warning >= 5.0:
                        logger.warning(
                            "No serial data received from Arduino on %s for %.1fs; "
                            "port is open, but check the flashed sketch, baud rate (%d), "
                            "USB data cable, and board reset/power.",
                            getattr(self._ser, "port", "unknown port"),
                            now - last_rx_time,
                            MOTION_SERIAL_BAUD,
                        )
                        last_no_data_warning = now
                    time.sleep(0.005)
                    continue

                raw = self._ser.readline()
                if not raw:
                    continue

                last_rx_time = time.monotonic()
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                if line.startswith("ENCODER:"):
                    parts = line[8:].split(",")
                    if len(parts) == 2:
                        try:
                            l_ticks = int(parts[0])
                            r_ticks = int(parts[1])
                        except ValueError:
                            logger.warning("Malformed encoder line from Arduino: %r", line)
                            continue

                        if not encoder_seen:
                            logger.info("Received encoder stream from Arduino: %s", line)
                            encoder_seen = True
                        self._last_encoder_time = time.monotonic()
                        self._update_odometry(l_ticks, r_ticks)
                    else:
                        logger.warning("Malformed encoder line from Arduino: %r", line)
                elif line.startswith("KEY:"):
                    key = line[4:].strip().upper()
                    if len(key) == 1 and key in "0123456789ABCD*#":
                        message = String()
                        message.data = key
                        self._keypad_pub.publish(message)
                        logger.info("Arduino keypad: %s", key)
                    else:
                        logger.warning("Malformed keypad line from Arduino: %r", line)
                else:
                    # Expose STATUS lines or a different firmware's output instead of
                    # silently discarding it; this distinguishes wrong protocol from no RX.
                    if line == "STATUS:STALL":
                        logger.error("Arduino cut motor power: encoder stall persisted after bounded PWM ramp")
                    logger.info("Arduino serial RX (non-ENCODER): %r", line[:160])
            except Exception as exc:
                now = time.monotonic()
                if now - last_read_error_log >= 2.0:
                    logger.error("Serial read failed on %s: %s", getattr(self._ser, "port", "unknown port"), exc)
                    last_read_error_log = now
                time.sleep(0.01)

    def _update_odometry(self, l_ticks: int, r_ticks: int):
        if self._first_enc:
            self._prev_l = l_ticks
            self._prev_r = r_ticks
            self._first_enc = False
            return

        delta_l_raw = l_ticks - self._prev_l
        delta_r_raw = r_ticks - self._prev_r

        # ป้องกัน Noise / Spike หลุดจาก Serial (เช่น ค่ากระโดดเกิน 2000 ticks หรือ 40cm ใน 100ms)
        if abs(delta_l_raw) > 2000 or abs(delta_r_raw) > 2000:
            logger.warning(f"Ignored encoder spike: dL_raw={delta_l_raw}, dR_raw={delta_r_raw}")
            self._prev_l = l_ticks
            self._prev_r = r_ticks
            return

        # สลับขั้ว Encoder ซ้าย/ขวา แยกอิสระเพื่อแก้ปัญหาข้างใดข้างหนึ่งนับถอยหลัง
        sign_l = -1.0 if self._invert_left_enc else 1.0
        sign_r = -1.0 if self._invert_right_enc else 1.0
        sign_lin = -1.0 if self._invert_linear else 1.0

        dl = sign_lin * sign_l * delta_l_raw * METERS_PER_TICK
        dr = sign_lin * sign_r * delta_r_raw * METERS_PER_TICK
        self._prev_l = l_ticks
        self._prev_r = r_ticks

        d = (dl + dr) / 2.0
        d_theta = (dr - dl) / self._effective_track_width
        if self._invert_odom_yaw:
            d_theta = -d_theta

        self._x += d * math.cos(self._theta + d_theta / 2.0)
        self._y += d * math.sin(self._theta + d_theta / 2.0)
        self._theta += d_theta

        # แสดง Log การขยับแบบเรียลไทม์ในเทอร์มินัลเมื่อล้อหมุน
        if abs(dl) > 0.0005 or abs(dr) > 0.0005:
            logger.info(
                f"[ODOM] dL={dl*100:+.1f}cm, dR={dr*100:+.1f}cm | "
                f"d={d*100:+.1f}cm, dTh={math.degrees(d_theta):+.1f}° | "
                f"Pos=({self._x:.2f}, {self._y:.2f})m Yaw={math.degrees(self._theta):.1f}°"
            )

    def _publish_odom_and_tf(self):
        # Do not publish synthetic zero odometry for a connected-but-silent or
        # garbled serial device; downstream real-robot preflight uses this as
        # evidence that encoder feedback is actually arriving.
        if self._ser and not self._arduino_ready:
            return

        now = self.get_clock().now().to_msg()
        half_theta = self._theta / 2.0
        qz = math.sin(half_theta)
        qw = math.cos(half_theta)

        transforms = []

        # 1. TF: odom -> base_footprint (สำหรับ slam_toolbox ที่ใช้ base_footprint)
        t_footprint = TransformStamped()
        t_footprint.header.stamp = now
        t_footprint.header.frame_id = "odom"
        t_footprint.child_frame_id = "base_footprint"
        t_footprint.transform.translation.x = self._x
        t_footprint.transform.translation.y = self._y
        t_footprint.transform.translation.z = 0.0
        t_footprint.transform.rotation.z = qz
        t_footprint.transform.rotation.w = qw
        transforms.append(t_footprint)

        # 2. TF: base_footprint -> base_link
        t_base = TransformStamped()
        t_base.header.stamp = now
        t_base.header.frame_id = "base_footprint"
        t_base.child_frame_id = "base_link"
        t_base.transform.translation.x = 0.0
        t_base.transform.translation.y = 0.0
        t_base.transform.translation.z = 0.0
        t_base.transform.rotation.w = 1.0
        transforms.append(t_base)

        for tr in transforms:
            self._tf_broadcaster.sendTransform(tr)

        # 4. Publish /odom Topic
        odom = OdomMsg()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self._odom_pub.publish(odom)


def main():
    if not HAS_ROS2:
        print("ERROR: ROS 2 (rclpy) is not available. Please source /opt/ros/jazzy/setup.bash first.")
        sys.exit(1)

    rclpy.init()

    # ค้นหาพอร์ต Arduino #1
    port = os.environ.get("MOTION_PORT", MOTION_SERIAL_PORT)
    ser = None
    if serial and port and port.lower() != "none":
        # LiDAR is on /dev/ttyUSB1; only try ACM ports for the Motion Arduino.
        for p in dict.fromkeys([port, "/dev/ttyACM0", "/dev/ttyACM1"]):
            if os.path.exists(p):
                try:
                    ser = serial.Serial(
                        p,
                        MOTION_SERIAL_BAUD,
                        timeout=min(SERIAL_TIMEOUT, ARDUINO_STARTUP_READ_TIMEOUT),
                    )
                    logger.info(f"Opened Arduino Motion Port on {p}")
                    break
                except Exception as e:
                    logger.warning(f"Could not open {p}: {e}")

    if ser:
        attempts = reset_and_wait_for_encoder(
            ser,
            timeout_seconds=ARDUINO_ENCODER_WAIT_SECONDS,
            attempts=ARDUINO_DTR_ATTEMPTS,
            dtr_pulse_seconds=ARDUINO_DTR_PULSE_SECONDS,
        )
        ready_attempt = next((i for i, result in enumerate(attempts, start=1) if result.frame), None)
        for attempt_number, result in enumerate(attempts, start=1):
            if result.reset_error:
                logger.warning("Arduino DTR reset attempt %d failed: %s", attempt_number, result.reset_error)
            if result.frame:
                logger.info(
                    "Arduino startup handshake received ENCODER:%d,%d at %d baud on DTR attempt %d/%d.",
                    result.frame[0],
                    result.frame[1],
                    MOTION_SERIAL_BAUD,
                    attempt_number,
                    ARDUINO_DTR_ATTEMPTS,
                )
                break
            if result.received_data:
                logger.warning(
                    "Arduino sent serial data at %d baud but no valid ENCODER frame arrived on DTR "
                    "attempt %d/%d; sample=%r.",
                    MOTION_SERIAL_BAUD,
                    attempt_number,
                    ARDUINO_DTR_ATTEMPTS,
                    result.samples[0] if result.samples else "<empty line>",
                )
            else:
                logger.warning(
                    "No serial data from Arduino at %d baud during DTR attempt %d/%d.",
                    MOTION_SERIAL_BAUD,
                    attempt_number,
                    ARDUINO_DTR_ATTEMPTS,
                )
            if attempt_number < len(attempts):
                logger.info("Pulsing Arduino DTR and retrying the encoder handshake once.")

        if ready_attempt is None:
            logger.error(
                "Arduino startup handshake failed after %d DTR attempts; SLAM will continue, "
                "but motion remains disabled until valid ENCODER frames arrive.",
                len(attempts),
            )
        ser.timeout = SERIAL_TIMEOUT

    yaw_offset = float(os.environ.get("LIDAR_YAW_OFFSET", "0.0"))
    node = SlamBridgeNode(ser=ser, yaw_offset_deg=yaw_offset)

    print("\n" + "=" * 55)
    print("  🗺️ SLAM & Teleop Test Bridge Running")
    print(f"  LiDAR Yaw Offset: {yaw_offset}°")
    print(
        "  Motor Control   : "
        + ("Waiting for valid Arduino encoder stream" if ser else "Unavailable (no Arduino serial port)")
    )
    print("=" * 55)
    print("  พร้อมให้ SLAM Toolbox และ Teleop Keyboard เชื่อมต่อแล้ว\n")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        if ser:
            try:
                ser.write(b"V:0.000,0.000\n")
                ser.flush()
                ser.close()
            except Exception:
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
