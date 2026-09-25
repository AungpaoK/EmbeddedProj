#!/usr/bin/env python3
"""Publish the fixed restaurant layout and real robot trail for RViz2."""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import yaml
from PIL import Image

from config import JUNCTION_X, TABLE1_Y, TABLE2_Y


class RestaurantVisualizer(Node):
    """Static map/marker publisher plus an odometry-backed path trail."""

    def __init__(self) -> None:
        super().__init__("restaurant_visualizer")
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._map_pub = self.create_publisher(OccupancyGrid, "/map", latched_qos)
        self._marker_pub = self.create_publisher(
            MarkerArray, "/scenario_markers", latched_qos
        )
        self._path_pub = self.create_publisher(RosPath, "/robot_path", 10)
        self._odom_sub = self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self._static_tf = tf2_ros.StaticTransformBroadcaster(self)

        self._map = self._load_map()
        self._path = RosPath()
        self._path.header.frame_id = "map"
        self._last_path_pose: tuple[float, float, float] | None = None

        self._publish_static_tf()
        self._publish_layout()
        self._refresh_timer = self.create_timer(1.0, self._publish_layout)
        self.get_logger().info(
            "Restaurant visualization ready: junction=%.2fm, table1=+%.2fm, table2=-%.2fm",
            JUNCTION_X,
            TABLE1_Y,
            TABLE2_Y,
        )

    def _load_map(self) -> OccupancyGrid:
        maps_dir = Path(__file__).resolve().parent.parent / "maps"
        yaml_path = maps_dir / "restaurant_map.yaml"
        with yaml_path.open("r", encoding="utf-8") as stream:
            metadata = yaml.safe_load(stream)

        image_path = maps_dir / metadata["image"]
        image = Image.open(image_path).convert("L")
        width, height = image.size
        origin = metadata["origin"]

        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info.resolution = float(metadata["resolution"])
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = float(origin[0])
        grid.info.origin.position.y = float(origin[1])
        grid.info.origin.position.z = float(origin[2])
        grid.info.origin.orientation.w = 1.0

        cells: list[int] = []
        for y in reversed(range(height)):
            for x in range(width):
                value = image.getpixel((x, y))
                if value >= 250:
                    cells.append(0)
                elif value <= 5:
                    cells.append(100)
                else:
                    cells.append(-1)
        grid.data = cells
        return grid

    def _publish_static_tf(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"
        transform.transform.rotation.w = 1.0
        self._static_tf.sendTransform(transform)

    def _publish_layout(self) -> None:
        now = self.get_clock().now().to_msg()
        self._map.header.stamp = now
        self._map_pub.publish(self._map)
        self._marker_pub.publish(self._make_markers(now))

    def _make_markers(self, stamp) -> MarkerArray:
        markers = MarkerArray()

        def add_box(
            marker_id: int,
            x: float,
            y: float,
            label: str,
            color: ColorRGBA,
            *,
            size_x: float = 0.8,
            size_y: float = 0.6,
            size_z: float = 0.7,
        ) -> None:
            box = Marker()
            box.header.stamp = stamp
            box.header.frame_id = "map"
            box.ns = "restaurant"
            box.id = marker_id
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = x
            box.pose.position.y = y
            box.pose.position.z = size_z / 2.0
            box.pose.orientation.w = 1.0
            box.scale.x = size_x
            box.scale.y = size_y
            box.scale.z = size_z
            box.color = color
            markers.markers.append(box)

            text = Marker()
            text.header.stamp = stamp
            text.header.frame_id = "map"
            text.ns = "restaurant_labels"
            text.id = marker_id + 100
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = size_z + 0.35
            text.pose.orientation.w = 1.0
            text.scale.z = 0.20
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = label
            markers.markers.append(text)

        add_box(
            1,
            -0.7,
            0.0,
            "Serve Station",
            ColorRGBA(r=0.2, g=0.8, b=0.2, a=0.85),
            size_x=0.6,
            size_y=1.2,
        )
        add_box(
            2,
            JUNCTION_X,
            0.0,
            "Junction",
            ColorRGBA(r=0.2, g=0.5, b=1.0, a=0.4),
            size_x=0.4,
            size_y=0.4,
            size_z=0.02,
        )
        add_box(
            3,
            JUNCTION_X,
            TABLE1_Y + 0.5,
            "Table 1",
            ColorRGBA(r=1.0, g=0.8, b=0.1, a=0.9),
        )
        add_box(
            4,
            JUNCTION_X,
            -TABLE2_Y - 0.5,
            "Table 2",
            ColorRGBA(r=1.0, g=0.4, b=0.1, a=0.9),
        )

        route = Marker()
        route.header.stamp = stamp
        route.header.frame_id = "map"
        route.ns = "planned_route"
        route.id = 200
        route.type = Marker.LINE_STRIP
        route.action = Marker.ADD
        route.scale.x = 0.035
        route.color = ColorRGBA(r=1.0, g=0.85, b=0.1, a=0.8)
        route.points = [
            Point(x=0.0, y=0.0, z=0.02),
            Point(x=JUNCTION_X, y=0.0, z=0.02),
            Point(x=JUNCTION_X, y=TABLE1_Y, z=0.02),
            Point(x=JUNCTION_X, y=-TABLE2_Y, z=0.02),
            Point(x=JUNCTION_X, y=0.0, z=0.02),
        ]
        markers.markers.append(route)
        return markers

    def _on_odom(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        siny = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny, cosy)

        if self._last_path_pose is not None:
            last_x, last_y, last_yaw = self._last_path_pose
            moved = math.hypot(position.x - last_x, position.y - last_y)
            turned = abs(math.atan2(math.sin(yaw - last_yaw), math.cos(yaw - last_yaw)))
            if moved < 0.005 and turned < math.radians(1.0):
                return

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = "map"
        pose.pose = msg.pose.pose
        self._path.header.stamp = msg.header.stamp
        self._path.poses.append(pose)
        if len(self._path.poses) > 1000:
            self._path.poses.pop(0)
        self._path_pub.publish(self._path)
        self._last_path_pose = (position.x, position.y, yaw)


def main() -> None:
    rclpy.init()
    node = RestaurantVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
