# Publish Odom node to ros2

#!/usr/bin/env python3

import math
import serial
import rclpy

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdometerNode(Node):
    def __init__(self):
        super().__init__("odometer_node")

        # --- Physical Robot Parameters (Synchronized) ---
        self.TICKS_PER_REV = 1080.0  # Ticks per full wheel revolution
        self.WHEEL_RADIUS = 0.035  # Wheel radius in meters (3.5 cm)
        self.WHEEL_BASE = 0.343  # Axle track width in meters (20 cm)

        # --- Serial Connection Setup ---
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        port = self.get_parameter("serial_port").get_parameter_value().string_value
        baud = self.get_parameter("baud_rate").get_parameter_value().integer_value

        try:
            self.ser = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f"Connected to Arduino on {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect on {port}: {e}")
            raise e

        # --- Publishers & Broadcasters ---
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Robot State Variables ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.totalDistance = 0.0  # Total path traveled

        self.lastLeftTicks = None
        self.lastRightTicks = None
        self.lastTime = self.get_clock().now()

        # Run reading and integration loop at ~20 Hz
        self.timer = self.create_timer(0.05, self.update_odometry)

    def update_odometry(self):
        if not self.ser.in_waiting:
            return

        try:
            # Assumes Arduino prints "left_ticks,right_ticks" per line
            arduino = self.ser.readline().decode("utf-8").strip()
            if not arduino:
                return

            encoderData = arduino.split(",")
            if len(encoderData) != 2:
                return

            leftTicks = int(encoderData[0])
            rightTicks = int(encoderData[1])

        except (ValueError, UnicodeDecodeError):
            return

        currentTime = self.get_clock().now()
        dt = (currentTime - self.lastTime).nanoseconds / 1e9

        # Initialize reference counts on first read
        if self.lastLeftTicks is None or self.lastRightTicks is None:
            self.lastLeftTicks = leftTicks
            self.lastRightTicks = rightTicks
            self.lastTime = currentTime
            return

        # 1. Delta Tick Calculation
        d_leftTicks = leftTicks - self.lastLeftTicks
        d_rightTicks = rightTicks - self.lastRightTicks

        self.lastLeftTicks = leftTicks
        self.lastRightTicks = rightTicks

        # 2. Convert Ticks to Meters
        metersPerTick = (2.0 * math.pi * self.WHEEL_RADIUS) / self.TICKS_PER_REV
        d_left = d_leftTicks * metersPerTick
        d_right = d_rightTicks * metersPerTick

        # 3. Differential Kinematics
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.WHEEL_BASE

        # 4. Integrate Spatial Pose (X, Y, Heading)
        if d_center != 0:
            self.x += d_center * math.cos(self.theta + (d_theta / 2.0))
            self.y += d_center * math.sin(self.theta + (d_theta / 2.0))
        self.theta += d_theta
        self.totalDistance += abs(d_center)  # Track overall distance

        v_x = d_center / dt if dt > 0 else 0.0
        v_theta = d_theta / dt if dt > 0 else 0.0

        # Yaw conversion to Quaternion
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        # 5. Broadcast Dynamic TF (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = currentTime.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # 6. Publish /odom Topic Message
        odom = Odometry()
        odom.header.stamp = currentTime.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v_x
        odom.twist.twist.angular.z = v_theta

        self.odom_pub.publish(odom)
        self.lastTime = currentTime


def main(args=None):
    rclpy.init(args=args)
    node = OdometerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
