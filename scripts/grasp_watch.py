#!/usr/bin/env python3
import rclpy
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray

rclpy.init()
node = rclpy.create_node("grasp_watch")
js = []
poses = []


def cb_js(msg):
    js.append((time.time(), msg))


def cb_poses(msg):
    poses.append((time.time(), msg))


node.create_subscription(JointState, "/joint_states", cb_js, 10)
node.create_subscription(PoseArray, "/world/arm_world/pose/info", cb_poses, 10)

print("sampling for 100s...", flush=True)
start = time.time()
while time.time() - start < 100:
    rclpy.spin_once(node, timeout_sec=0.5)

js.sort(key=lambda x: x[0])
poses.sort(key=lambda x: x[0])


def state_at(t):
    best_js, best_po = None, None
    for st, m in js:
        if st >= t:
            best_js = (st, {n: v for n, v in zip(m.name, m.position)})
            break
    for st, pa in poses:
        if st >= t:
            best_po = pa
            break
    return best_js, best_po


for mark in range(0, 100, 10):
    t0 = start + mark
    sjs, spo = state_at(t0)
    if not sjs or spo is None:
        print(f"t+{mark:3d}s (no data)")
        continue
    d = sjs[1]
    blocks = [p.position for p in spo.poses
              if 0.70 < p.position.z < 0.95 and abs(p.position.x) < 0.5
              and abs(p.position.y) < 0.5]
    lf = d.get("left_finger_joint", 0.0)
    rf = d.get("right_finger_joint", 0.0)
    print(f"t+{mark:3d}s lf={lf:+.2f} rf={rf:+.2f} j1={d['joint_1']:+.2f} "
          f"j2={d['joint_2']:+.2f} | blocks: " +
          ", ".join(f"({p.x:+.2f},{p.y:+.2f},{p.z:+.2f})" for p in blocks))
rclpy.shutdown()
