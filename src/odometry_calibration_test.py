#!/usr/bin/env python3
"""Interactive wheel-odometry calibration tests for the real robot.

This tool owns the motion Arduino serial port directly.  Stop ``start_robot.sh``
and any other process using the motion Arduino before running it.

The tests deliberately use the continuous ``V:left,right`` protocol used by the
ROS delivery path, rather than the legacy blocking FORWARD/TURN commands.
They collect raw encoder frames and an encoder-only pose estimate, then ask the
operator for measurements that encoders cannot provide (real distance, lateral
drift, and real turn angle).

Examples:
    python3 src/odometry_calibration_test.py --test straight --condition empty
    python3 src/odometry_calibration_test.py --test roundtrip --condition payload
    python3 src/odometry_calibration_test.py --test turn --condition empty
    python3 src/odometry_calibration_test.py --test all --condition empty
    python3 src/odometry_calibration_test.py --test all --repeats 5 --condition payload

The default signs match the current ROS launcher: INVERT_LINEAR=1 and
INVERT_ODOM_YAW=0.  If the robot moves in the wrong direction, stop it with
Ctrl-C and rerun with explicit sign overrides instead of continuing.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import subprocess
import sys
import time
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path

try:
    import serial
except ImportError as exc:  # pragma: no cover - depends on the target Pi
    raise SystemExit("pySerial is required. Install/source the robot environment first.") from exc

from arduino_serial import parse_encoder_line, reset_and_wait_for_encoder


ROOT = Path(__file__).resolve().parents[1]


@dataclass
class RobotConfig:
    wheel_radius_m: float = 0.065
    wheel_base_m: float = 0.343
    track_width_factor: float = 1.185
    ticks_per_rev: float = 1920.0
    left_tick_scale: float = 1.0
    right_tick_scale: float = 1.0
    forward_command_sign: int = -1
    encoder_forward_sign: int = -1

    @property
    def meters_per_tick(self) -> float:
        return 2.0 * math.pi * self.wheel_radius_m / self.ticks_per_rev

    @property
    def effective_track_width_m(self) -> float:
        return self.wheel_base_m * self.track_width_factor


@dataclass
class EncoderPose:
    x_m: float = 0.0
    y_m: float = 0.0
    theta_rad: float = 0.0
    left_distance_m: float = 0.0
    right_distance_m: float = 0.0
    raw_left_ticks: int = 0
    raw_right_ticks: int = 0

    @property
    def theta_deg(self) -> float:
        return math.degrees(self.theta_rad)

    @property
    def center_distance_m(self) -> float:
        return (self.left_distance_m + self.right_distance_m) / 2.0


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class PoseIntegrator:
    """Integrate the same differential-drive model used by slam_bridge."""

    def __init__(self, config: RobotConfig) -> None:
        self.config = config
        self.pose = EncoderPose()
        self._last_left: int | None = None
        self._last_right: int | None = None

    def update(self, raw_left: int, raw_right: int) -> bool:
        if self._last_left is None or self._last_right is None:
            self._last_left = raw_left
            self._last_right = raw_right
            self.pose.raw_left_ticks = raw_left
            self.pose.raw_right_ticks = raw_right
            return False

        delta_left = raw_left - self._last_left
        delta_right = raw_right - self._last_right
        self._last_left = raw_left
        self._last_right = raw_right

        sign = self.config.encoder_forward_sign
        meters_per_tick = self.config.meters_per_tick
        dl = sign * delta_left * meters_per_tick * self.config.left_tick_scale
        dr = sign * delta_right * meters_per_tick * self.config.right_tick_scale

        center = (dl + dr) / 2.0
        dtheta = (dr - dl) / self.config.effective_track_width_m
        self.pose.x_m += center * math.cos(self.pose.theta_rad + dtheta / 2.0)
        self.pose.y_m += center * math.sin(self.pose.theta_rad + dtheta / 2.0)
        self.pose.theta_rad = normalize_angle(self.pose.theta_rad + dtheta)
        self.pose.left_distance_m += dl
        self.pose.right_distance_m += dr
        self.pose.raw_left_ticks = raw_left
        self.pose.raw_right_ticks = raw_right
        return True


class MotionTestRunner:
    """Own the Arduino serial link and collect encoder telemetry."""

    def __init__(self, ser, config: RobotConfig, test_name: str, csv_path: Path) -> None:
        self.ser = ser
        self.config = config
        self.test_name = test_name
        self.csv_path = csv_path
        self.pose = PoseIntegrator(config)
        self._started_at = time.monotonic()
        self._last_status = ""
        self._csv_file = csv_path.open("w", newline="", encoding="utf-8")
        self._csv = csv.DictWriter(
            self._csv_file,
            fieldnames=[
                "test",
                "elapsed_s",
                "raw_left_ticks",
                "raw_right_ticks",
                "left_distance_m",
                "right_distance_m",
                "center_distance_m",
                "x_m",
                "y_m",
                "theta_deg",
                "command_left_mps",
                "command_right_mps",
            ],
        )
        self._csv.writeheader()
        self._csv_file.flush()

    def close(self) -> None:
        self._csv_file.close()

    def _record_frame(self, raw_left: int, raw_right: int, left_cmd: float, right_cmd: float) -> None:
        self.pose.update(raw_left, raw_right)
        p = self.pose.pose
        self._csv.writerow(
            {
                "test": self.test_name,
                "elapsed_s": f"{time.monotonic() - self._started_at:.3f}",
                "raw_left_ticks": raw_left,
                "raw_right_ticks": raw_right,
                "left_distance_m": f"{p.left_distance_m:.6f}",
                "right_distance_m": f"{p.right_distance_m:.6f}",
                "center_distance_m": f"{p.center_distance_m:.6f}",
                "x_m": f"{p.x_m:.6f}",
                "y_m": f"{p.y_m:.6f}",
                "theta_deg": f"{p.theta_deg:.4f}",
                "command_left_mps": f"{left_cmd:.4f}",
                "command_right_mps": f"{right_cmd:.4f}",
            }
        )
        self._csv_file.flush()

    def drain(self, left_cmd: float = 0.0, right_cmd: float = 0.0) -> int:
        """Read all complete lines currently available and record encoder frames."""
        frames = 0
        while self.ser.in_waiting:
            raw = self.ser.readline()
            if not raw:
                break
            frame = parse_encoder_line(raw)
            if frame is not None:
                self._record_frame(frame[0], frame[1], left_cmd, right_cmd)
                frames += 1
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if line.startswith("STATUS:"):
                self._last_status = line
                if line in {"STATUS:STALL", "STATUS:ERROR"}:
                    print(f"\nArduino reported {line}", file=sys.stderr)
        return frames

    def send_wheels(self, left_mps: float, right_mps: float) -> None:
        self.ser.write(f"V:{left_mps:.3f},{right_mps:.3f}\n".encode("utf-8"))
        self.ser.flush()

    def stop(self) -> None:
        try:
            self.ser.write(b"V:0.000,0.000\n")
            self.ser.flush()
        except Exception:
            pass
        deadline = time.monotonic() + 0.6
        while time.monotonic() < deadline:
            self.drain()
            time.sleep(0.02)

    def run_velocity(self, linear_v: float, angular_w: float, duration_s: float, *, label: str) -> EncoderPose:
        left = linear_v - angular_w * self.config.wheel_base_m / 2.0
        right = linear_v + angular_w * self.config.wheel_base_m / 2.0
        print(
            f"{label}: V left={left:+.3f} m/s, right={right:+.3f} m/s "
            f"for {duration_s:.1f}s"
        )

        deadline = time.monotonic() + duration_s
        next_command = time.monotonic()
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_command:
                self.send_wheels(left, right)
                next_command = now + 0.10
            self.drain(left, right)
            time.sleep(0.01)

        self.stop()
        return self.pose.pose


def find_motion_port(requested: str | None) -> str:
    if requested and requested.lower() not in {"auto", ""}:
        return requested

    candidates = sorted(Path("/dev/serial/by-id").glob("*Arduino*"))
    if len(candidates) == 1:
        return str(candidates[0])
    if len(candidates) > 1:
        raise SystemExit("พบ Arduino หลายตัว กรุณาระบุ --port ให้ชัดเจน")

    for candidate in ("/dev/ttyACM0", "/dev/ttyACM1"):
        if Path(candidate).exists():
            return candidate
    raise SystemExit("ไม่พบ Motion Arduino; ใช้ --port /dev/ttyACM0 หรือพอร์ตจริงของบอร์ด")


def read_latest_encoder_frame(ser, wait_seconds: float = 0.5) -> tuple[int, int]:
    """Read a fresh encoder baseline after the operator repositions the robot."""
    deadline = time.monotonic() + wait_seconds
    latest: tuple[int, int] | None = None
    while time.monotonic() < deadline:
        if ser.in_waiting:
            raw = ser.readline()
            frame = parse_encoder_line(raw)
            if frame is not None:
                latest = frame
        else:
            time.sleep(0.02)
    if latest is None:
        raise SystemExit("ไม่พบ ENCODER frame หลังจัดหุ่นใหม่; ตรวจสายหรือ Arduino")
    return latest


def check_conflicting_sessions(session: str, skip: bool) -> None:
    if skip or not shutil_which("tmux"):
        return
    for candidate in (session, "slam", "pos"):
        result = subprocess.run(
            ["tmux", "has-session", "-t", candidate],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if result.returncode == 0:
            raise SystemExit(
                f"พบ tmux session '{candidate}' กำลังทำงานอยู่; "
                "หยุด start_robot/slam ก่อน แล้วค่อยรัน calibration test"
            )


def shutil_which(command: str) -> str | None:
    """Small local wrapper to keep the session check easy to test."""
    from shutil import which

    return which(command)


def ask_float(prompt: str, *, default: float | None = None, allow_blank: bool = True) -> float | None:
    suffix = f" [{default}]" if default is not None else ""
    while True:
        raw = input(f"{prompt}{suffix}: ").strip()
        if not raw and allow_blank:
            return default
        try:
            return float(raw)
        except ValueError:
            print("กรุณาใส่ตัวเลข เช่น 1.98 หรือ 0.12")


def ask_note() -> str:
    return input("หมายเหตุเพิ่มเติม (เว้นว่างได้): ").strip()


def wait_for_operator(message: str) -> None:
    input(f"\n{message}\nกด Enter เพื่อเริ่ม หรือ Ctrl-C เพื่อยกเลิก...")


def make_result(
    *,
    test: str,
    condition: str,
    pose: EncoderPose,
    csv_path: Path,
    measurements: dict,
    note: str,
) -> dict:
    result = {
        "test": test,
        "condition": condition,
        "timestamp": datetime.now().astimezone().isoformat(timespec="seconds"),
        "encoder_pose": asdict(pose),
        "measurements": measurements,
        "notes": note,
        "telemetry_csv": str(csv_path),
    }
    if "actual_distance_m" in measurements and measurements["actual_distance_m"] is not None:
        result["distance_error_m"] = pose.center_distance_m - measurements["actual_distance_m"]
    if "actual_turn_deg" in measurements and measurements["actual_turn_deg"] is not None:
        result["turn_error_deg"] = abs(pose.theta_deg) - abs(measurements["actual_turn_deg"])
    return result


def run_straight(runner: MotionTestRunner, args: argparse.Namespace) -> dict:
    distance = args.distance_m
    duration = distance / args.speed_mps
    wait_for_operator(
        f"วางหุ่นที่จุดเริ่ม วิ่งตรงตามแนวเส้น {distance:.2f}m "
        f"และเคลียร์พื้นที่ด้านหน้า ระยะเวลาประมาณ {duration:.1f}s"
    )
    pose = runner.run_velocity(
        args.forward_sign * args.speed_mps,
        0.0,
        duration,
        label="STRAIGHT",
    )
    print(f"Encoder: distance={pose.center_distance_m:.4f}m, y={pose.y_m:+.4f}m")
    actual = ask_float("วัดระยะที่ตัวหุ่นเคลื่อนจริง (เมตร)")
    lateral = ask_float("วัดระยะเบี้ยวด้านข้างจากเส้นเริ่ม (เมตร)")
    return make_result(
        test="straight",
        condition=args.condition,
        pose=pose,
        csv_path=runner.csv_path,
        measurements={"actual_distance_m": actual, "actual_lateral_drift_m": lateral},
        note=ask_note(),
    )


def run_roundtrip(runner: MotionTestRunner, args: argparse.Namespace) -> dict:
    distance = args.distance_m
    one_way_duration = distance / args.speed_mps
    wait_for_operator(
        f"วางหุ่นที่จุดเริ่ม วิ่งหน้า {distance:.2f}m แล้วถอยกลับจุดเดิม "
        f"จำนวน {args.cycles} รอบ พื้นที่ต้องโล่งทั้งสองทิศ"
    )
    for cycle in range(1, args.cycles + 1):
        print(f"รอบ {cycle}/{args.cycles}: เดินหน้า")
        runner.run_velocity(args.forward_sign * args.speed_mps, 0.0, one_way_duration, label="FORWARD")
        time.sleep(args.pause_s)
        print(f"รอบ {cycle}/{args.cycles}: ถอยกลับ")
        runner.run_velocity(-args.forward_sign * args.speed_mps, 0.0, one_way_duration, label="REVERSE")
        time.sleep(args.pause_s)

    pose = runner.pose.pose
    print(f"Encoder final: x={pose.x_m:+.4f}m, y={pose.y_m:+.4f}m, theta={pose.theta_deg:+.2f}deg")
    actual_x_error = ask_float("วัดระยะคลาดตามแนวเส้นจากจุดเริ่ม (เมตร)")
    actual_lateral = ask_float("วัด lateral drift จากจุดเริ่ม (เมตร)")
    return make_result(
        test="roundtrip",
        condition=args.condition,
        pose=pose,
        csv_path=runner.csv_path,
        measurements={
            "cycles": args.cycles,
            "actual_longitudinal_error_m": actual_x_error,
            "actual_lateral_drift_m": actual_lateral,
        },
        note=ask_note(),
    )


def run_turn(runner: MotionTestRunner, args: argparse.Namespace) -> dict:
    target_deg = args.turn_degrees
    target_rad = math.radians(target_deg)
    duration = abs(target_rad / args.turn_rate_rad_s)
    wait_for_operator(
        f"วางหุ่นบนจุดหมุน ทำเครื่องหมายทิศหน้าหุ่น และเคลียร์พื้นที่รอบตัว "
        f"จะหมุนประมาณ {target_deg:.0f}° ใช้เวลาประมาณ {duration:.1f}s"
    )
    pose = runner.run_velocity(
        0.0,
        args.turn_sign * args.turn_rate_rad_s,
        duration,
        label="TURN",
    )
    print(f"Encoder: turn={pose.theta_deg:+.2f}deg")
    actual = ask_float("วัดมุมที่หมุนได้จริง (องศา ไม่ต้องใส่เครื่องหมาย)")
    return make_result(
        test="turn",
        condition=args.condition,
        pose=pose,
        csv_path=runner.csv_path,
        measurements={"actual_turn_deg": actual},
        note=ask_note(),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Interactive odometry calibration tests")
    parser.add_argument("--test", choices=("straight", "roundtrip", "turn", "all"), default="all")
    parser.add_argument(
        "--repeats",
        type=int,
        default=1,
        help="repeat each selected test after operator repositions the robot",
    )
    parser.add_argument("--condition", default="empty", help="payload label, e.g. empty or payload")
    parser.add_argument("--port", default=os.environ.get("MOTION_PORT", "auto"))
    parser.add_argument("--baud", type=int, default=int(os.environ.get("MOTION_SERIAL_BAUD", "115200")))
    parser.add_argument("--output-dir", default="calibration_logs")
    parser.add_argument("--session", default=os.environ.get("ROBOT_TMUX_SESSION", "food-robot"))
    parser.add_argument("--skip-session-check", action="store_true")
    parser.add_argument("--yes", action="store_true", help="skip the initial safety confirmation")
    parser.add_argument("--distance-m", type=float, default=2.0)
    parser.add_argument("--cycles", type=int, default=10)
    parser.add_argument("--speed-mps", type=float, default=0.12)
    parser.add_argument("--turn-degrees", type=float, default=360.0)
    parser.add_argument("--turn-rate-rad-s", type=float, default=0.25)
    parser.add_argument("--pause-s", type=float, default=1.0)
    parser.add_argument("--forward-sign", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--encoder-sign", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--turn-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--wheel-radius-m", type=float, default=0.065)
    parser.add_argument("--wheel-base-m", type=float, default=0.343)
    parser.add_argument(
        "--track-width-factor",
        type=float,
        default=float(os.environ.get("ODOM_TRACK_WIDTH_FACTOR", "1.185")),
    )
    parser.add_argument("--ticks-per-rev", type=float, default=1920.0)
    parser.add_argument(
        "--left-tick-scale",
        type=float,
        default=float(os.environ.get("LEFT_TICK_SCALE", "1.0")),
    )
    parser.add_argument(
        "--right-tick-scale",
        type=float,
        default=float(os.environ.get("RIGHT_TICK_SCALE", "1.0")),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if (
        args.distance_m <= 0
        or args.cycles < 1
        or args.repeats < 1
        or args.speed_mps <= 0
        or args.turn_rate_rad_s <= 0
    ):
        raise SystemExit("ระยะ, cycles, repeats, speed และ turn rate ต้องมากกว่า 0")
    if args.speed_mps > 0.20 or args.turn_rate_rad_s > 0.50:
        raise SystemExit("ค่าเริ่มต้นปลอดภัยไม่ควรเกิน speed 0.20 m/s หรือ turn rate 0.50 rad/s")

    check_conflicting_sessions(args.session, args.skip_session_check)
    port = find_motion_port(args.port)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    summary_path = output_dir / f"odometry_{args.condition}_{stamp}.json"

    config = RobotConfig(
        wheel_radius_m=args.wheel_radius_m,
        wheel_base_m=args.wheel_base_m,
        track_width_factor=args.track_width_factor,
        ticks_per_rev=args.ticks_per_rev,
        left_tick_scale=args.left_tick_scale,
        right_tick_scale=args.right_tick_scale,
        forward_command_sign=args.forward_sign,
        encoder_forward_sign=args.encoder_sign,
    )

    print("=" * 64)
    print("ODOMETRY CALIBRATION TEST")
    print(f"Port: {port} @ {args.baud} | condition: {args.condition}")
    print(f"Test: {args.test} | output: {summary_path}")
    print("ต้องหยุด start_robot/slam_bridge และยกขาตั้งล้อ/พื้นที่เสี่ยงให้เรียบร้อย")
    print("Ctrl-C จะส่งคำสั่งหยุดมอเตอร์ทันที")
    print("=" * 64)
    if not args.yes:
        input("พิมพ์ Enter เมื่อยืนยันว่าพื้นที่ปลอดภัยและไม่มีโปรเซสอื่นใช้ Arduino...")

    try:
        ser = serial.Serial(port, args.baud, timeout=0.05)
    except (serial.SerialException, OSError) as exc:
        raise SystemExit(f"เปิด Motion Arduino ไม่ได้ที่ {port}: {exc}") from exc

    results: list[dict] = []
    runner: MotionTestRunner | None = None
    try:
        attempts = reset_and_wait_for_encoder(ser, timeout_seconds=5.0, attempts=2)
        frame = next((attempt.frame for attempt in attempts if attempt.frame is not None), None)
        if frame is None:
            raise SystemExit("ไม่พบ ENCODER frame จาก Arduino; ตรวจ baud, firmware และสาย USB")
        print(f"Encoder ready: raw L={frame[0]}, R={frame[1]}")

        selected = [args.test] if args.test != "all" else ["straight", "roundtrip", "turn"]
        total_runs = len(selected) * args.repeats
        run_number = 0
        for test_name in selected:
            for repeat_index in range(args.repeats):
                run_number += 1
                csv_path = output_dir / (
                    f"odometry_{args.condition}_{stamp}_{test_name}_r{repeat_index + 1}.csv"
                )
                print(f"\nCalibration run {run_number}/{total_runs}: {test_name} #{repeat_index + 1}")
                runner = MotionTestRunner(ser, config, test_name, csv_path)
                runner.pose.update(frame[0], frame[1])
                if test_name == "straight":
                    results.append(run_straight(runner, args))
                elif test_name == "roundtrip":
                    results.append(run_roundtrip(runner, args))
                else:
                    results.append(run_turn(runner, args))
                runner.stop()
                final_frame = (
                    runner.pose.pose.raw_left_ticks,
                    runner.pose.pose.raw_right_ticks,
                )
                runner.close()
                runner = None

                if run_number < total_runs:
                    wait_for_operator("จัดหุ่นกลับตำแหน่งเริ่มต้นและเตรียมการทดสอบถัดไป")
                    frame = read_latest_encoder_frame(ser)
                else:
                    frame = final_frame

        summary = {
            "config": asdict(config),
            "port": port,
            "baud": args.baud,
            "results": results,
            "notes": "ทดสอบด้วยคำสั่ง V continuous; ค่าระยะ/ดริฟต์/มุมจริงมาจากผู้ปฏิบัติงาน",
        }
        summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
        print("\nเสร็จแล้ว")
        print(f"Summary: {summary_path}")
        print(f"Telemetry CSV: {output_dir}")
        return 0
    except KeyboardInterrupt:
        print("\nยกเลิกการทดสอบ")
        return 130
    finally:
        if runner is not None:
            runner.stop()
            runner.close()
        try:
            ser.write(b"V:0.000,0.000\n")
            ser.flush()
        except Exception:
            pass
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
