#!/usr/bin/env python3
"""Grab one frame from each arm camera and save it as JPEG.

Runs inside the arm_ros container against the live ROS 2 stack.

Usage:
  python3 /project/scripts/snap_cameras.py [outdir]

Exits non-zero if any camera never published a frame.
"""

import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

TOPICS = {
    "/camera_table/image_raw": "camera_table.jpg",
    "/camera_stick/image_raw": "camera_stick.jpg",
    "/camera_ee/image_raw": "camera_ee.jpg",
}


class Snapper(Node):

    def __init__(self):
        super().__init__("snap_cameras")
        self.frames = {}
        for topic in TOPICS:
            self.create_subscription(
                Image, topic, self._make_cb(topic),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def _make_cb(self, topic):
        def cb(msg):
            if topic in self.frames:
                return
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                (msg.height, msg.width, 3))
            self.frames[topic] = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            self.get_logger().info(f"{topic}: {msg.width}x{msg.height}")
        return cb

    def save(self, outdir):
        for topic, fname in TOPICS.items():
            if topic not in self.frames:
                continue
            path = os.path.join(outdir, fname)
            cv2.imwrite(path, self.frames[topic])
            self.get_logger().info(f"saved {path}")


def main():
    rclpy.init()
    outdir = sys.argv[1] if len(sys.argv) > 1 else "/tmp"
    os.makedirs(outdir, exist_ok=True)
    node = Snapper()
    deadline = time.time() + 20.0
    while time.time() < deadline and len(node.frames) < len(TOPICS):
        rclpy.spin_once(node, timeout_sec=0.5)
    node.save(outdir)
    missing = [t for t in TOPICS if t not in node.frames]
    for topic in missing:
        node.get_logger().error(f"no frame from {topic}")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if missing else 0)


if __name__ == "__main__":
    main()
