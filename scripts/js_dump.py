#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import JointState

rclpy.init()
node = rclpy.create_node("js_dump")
got = []


def cb(msg):
    got.append(msg)


node.create_subscription(JointState, "/joint_states", cb, 10)
import time
deadline = time.time() + 6
while time.time() < deadline and not got:
    rclpy.spin_once(node, timeout_sec=0.2)

if got:
    m = got[0]
    print("stamp:", m.header.stamp.sec, m.header.stamp.nanosec)
    for n, p in zip(m.name, m.position):
        print(f"{n:20s} {p:+.6f}")
else:
    print("no joint_states received")
rclpy.shutdown()
