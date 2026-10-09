#!/usr/bin/env python3
"""Explicit M1 known-geometry source. No sensor perception is claimed."""

import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point32
from qr_interfaces.msg import SupportRegionArray, SupportRegion
from visualization_msgs.msg import MarkerArray, Marker
from course import fixture


class Surfaces(Node):
    def __init__(self, height):
        super().__init__("qr_known_surfaces")
        self.spec = fixture(height)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(SupportRegionArray, "/qr/support_regions", qos)
        self.markers = self.create_publisher(MarkerArray, "/qr/support_markers", qos)
        self.create_timer(0.1, self.publish)

    def publish(self):
        msg = SupportRegionArray()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.revision = 1
        markers = MarkerArray()
        for p in self.spec["pads"]:
            region = SupportRegion()
            region.id = p["id"]
            region.known = True
            region.confidence = 1.0
            region.normal.z = 1.0
            region.observed_at = msg.header.stamp
            x, y = p["center"]
            z = self.spec["height"]
            region.polygon.points = [
                Point32(x=x + dx, y=y + dy, z=z)
                for dx, dy in [(-0.1, -0.1), (0.1, -0.1), (0.1, 0.1), (-0.1, 0.1)]
            ]
            msg.regions.append(region)
            m = Marker()
            m.header = msg.header
            m.ns = "known_support"
            m.id = p["id"]
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = z - 0.001
            m.pose.orientation.w = 1.0
            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.002
            m.color.r = 0.1
            m.color.g = 0.8
            m.color.a = 0.5
            markers.markers.append(m)
        self.pub.publish(msg)
        self.markers.publish(markers)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--height", type=float, default=0.0)
    a, rest = p.parse_known_args()
    rclpy.init(args=rest)
    n = Surfaces(a.height)
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()
