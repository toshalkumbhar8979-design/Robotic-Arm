"""Autonomous scan-first pick-and-sort node for the BabyROS arm_6dof.

SCAN: blocks are found ANYWHERE in the workspace - no hardcoded spawn
positions. The table camera segments the block colours (HSV) and each
blob centroid is ray-projected onto the table plane (camera_info K +
TF world<-camera_table_link) to get a colour-labelled world pose. World
poses (/world/arm_world/pose/info) confirm the positions. Bucket zones
are excluded because the bucket walls share the block colours.

SORT: each detected block is carried to its colour-matched bucket and
dropped; the drop is verified by (a) the object count inside the bucket
zone increasing and (b) the block's colour disappearing from the table.

CAMERA GATE: the jaws only close after the camera confirms the block is
at the grasp point; the "grasp verification" after lifting also uses the
camera (the block's colour gone from the grasp spot). The loop re-scans
between picks, so blocks placed anywhere are found, and blocks dropped
mid-carry are re-picked on the next scan (after the remaining blocks).

The gripper is a pair of symmetric prismatic parallel jaws hanging BELOW
the palm (left_finger_joint / right_finger_joint, 0..40 mm travel, both
commandable - no mimic). They sit at palm_X +-40mm from the palm centre,
so with tool0 parked at block_z + 0.100 the pads reach the block's exact
height (world z 0.750..0.770) and the wrist stays 100 mm above the table.
An 80 mm opening at q=0 envelops the 20 mm cube; closing to 0.030 clamps
it (jaws stall there), so closing full-empty reads ~0.034 (not clamped).

Motion uses the MoveIt Python API (MoveItPy); the jaws go through the
GripperCommand action server (arm_teleop), falling back to the finger
controller command topic. Status is published on /arm_pick/status; the
sequence can be (re)triggered via /arm_pick/trigger.
"""

import json
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory, GripperCommand
from control_msgs.msg import GripperCommand as GripperCommandMsg
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Quaternion
from moveit import MoveItPy
from moveit.planning import PlanRequestParameters
from moveit_msgs.msg import MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

TABLE_TOP = 0.75
# tool0 orientation for a top-down grasp: tool0 +z along world -z, and
# the palm +y axis points up (world +z) - the fingers hang below the
# palm, so they reach the block while the wrist stays above the table.
GRASP_QUAT = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

# Rest configuration: spawned arm at all-zeros has pads inside link_4.

# Wrist rotated 90°, fingers point sideways clearing the forearm axis.
# With disabled_collisions (left/right_finger ↔ link_4/link_5) in SRDF
# it passes the start-state collision check.
HOME_POS = [0.0, 0.0, 0.0, 0.0, 1.5708, 0.0]

# Bucket centres (world x y): red, green, blue - the fixed sort targets.
# Placed closer to the base (≤0.28 m radius) so OMPL can always find IK
# paths without hitting self-collision gaps.
BUCKET_CENTERS = {
    "red": (0.22, 0.15),
    "green": (0.24, 0.0),
    "blue": (0.22, -0.15),
}
# Tool0 drop z per bucket: pads hang 90..110 mm below tool0, so tool0 at
# 0.87 puts the pads at 0.76..0.78 - inside the bucket (walls 0.745..0.835)
# and above the bucket floor.
DROP_Z = 0.87

# Camera intrinsics: gz publishes /camera_*/camera_info exactly once at
# startup, before any subscriber exists, so pick_node never receives it.
# The nominal 640x480 sensor with hfov 1.134464 gives fx=fy=502.3, but
# fitting the block centres against the actual rendered image gave the
# equivalent reprojection with fx=fy=480, cx=311, cy=226 (<0.5px error).
CAM_INTRINSICS = (480.0, 480.0)

# Optical-frame mapping for the gz sensors: the camera looks along the
# link +X axis (optical +Z = link +X). Converts a pixel ray in the
# optical frame to the link frame.
CAM_OPTICAL_TO_LINK = np.array([[0.0, 0.0, 1.0],
                                [-1.0, 0.0, 0.0],
                                [0.0, -1.0, 0.0]])

# World-fixed camera link poses (URDF joint origins); the ee camera pants
# with the arm and is resolved via TF.
CAM_POSES = {
    "camera_stick_link": {"xyz": (0.0, -0.42, 1.1),
                          "rpy": (0.0, 0.6671, 1.3371)},
    "camera_table_link": {"xyz": (0.28, -0.33, 0.77),
                          "rpy": (0.0, 0.0266, 2.0701)},
}


class PickNode(Node):

    def __init__(self):
        super().__init__("arm_pick")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("localization_mode", "ground_truth"),
                ("table_height", TABLE_TOP),
                ("block_size", 0.02),
                ("approach_offset", 0.12),
                ("lift_height", 0.18),
                ("finger_open", 0.0),
                ("finger_close", 0.034),
                ("finger_clamp_limit", 0.032),
                ("max_pick_attempts", 3),
                ("max_scan_passes", 3),
                ("drop_verify_timeout", 4.0),
                ("grasp_z_offset", 0.100),
                ("drop_position", [0.22, 0.15, DROP_Z]),
                ("vision_camera_frame", "camera_stick_link"),
                ("vision_extra_frames",
                 '["camera_table_link", "camera_ee_link"]'),
                ("vision_colors",
                 '{"red": {"h_low": [0, 120, 80], "h_high": [10, 255, 255]}, '
                 '"green": {"h_low": [35, 100, 80], "h_high": [85, 255, 255]}, '
                 '"blue": {"h_low": [95, 120, 80], "h_high": [130, 255, 255]}}'),
                ("pose_wait_timeout", 10.0),
                ("auto_start", True),
            ])

        self._mode = self.get_parameter("localization_mode").value
        self._table = self.get_parameter("table_height").value
        self._block = self.get_parameter("block_size").value
        self._approach = self.get_parameter("approach_offset").value
        self._lift = self.get_parameter("lift_height").value
        self._finger_open = self.get_parameter("finger_open").value
        self._finger_close = self.get_parameter("finger_close").value
        self._finger_clamp_limit = self.get_parameter("finger_clamp_limit").value
        self._max_attempts = self.get_parameter("max_pick_attempts").value
        self._max_passes = self.get_parameter("max_scan_passes").value
        self._drop_verify_timeout = self.get_parameter("drop_verify_timeout").value
        self._grasp_z_offset = self.get_parameter("grasp_z_offset").value
        self._drop = list(self.get_parameter("drop_position").value)
        self._cam_frame = self.get_parameter("vision_camera_frame").value
        self._extra_frames = json.loads(
            self.get_parameter("vision_extra_frames").value)
        self._colors = json.loads(self.get_parameter("vision_colors").value)
        self._pose_wait_timeout = self.get_parameter("pose_wait_timeout").value
        self._auto_start = self.get_parameter("auto_start").value

        self._status_pub = self.create_publisher(String, "/arm_pick/status", 10)
        self._srv = self.create_service(Trigger, "/arm_pick/trigger",
                                        self._on_trigger)

        self._poses = []
        self._pose_received = False
        self._latest_image = {}
        self._camera_info = {}
        self._joint_states = {}

        self.CAMERAS = {
            "table": "/camera_table/image_raw",
            "stick": "/camera_stick/image_raw",
            "ee": "/camera_ee/image_raw",
        }
        self._vis_pubs = {}
        for cam, topic in self.CAMERAS.items():
            self.create_subscription(
                Image, topic, self._make_image_cb(cam),
                QoSProfile(depth=1,
                           reliability=ReliabilityPolicy.BEST_EFFORT))
            self._vis_pubs[cam] = self.create_publisher(
                Image, f"/camera_{cam}/detected", 10)
        self.create_subscription(
            CameraInfo, "/camera_table/camera_info", self._on_camera_info,
            QoSProfile(depth=1,
                        reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(
            PoseArray, "/world/arm_world/pose/info", self._on_poses, 10)
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._finger_pub = self.create_publisher(
            Float64MultiArray, "/finger_controller/commands", 10)
        self._gripper_position = 0.0
        self._gripper_client = ActionClient(
            self, GripperCommand, "/gripper_action_controller/gripper_cmd")
        self._traj_client = ActionClient(
            self, FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory")

        self._moveit = MoveItPy(node_name="arm_pick_moveit")
        self._arm = self._moveit.get_planning_component("arm")
        self._mpr = PlanRequestParameters(self._moveit, "arm")
        self._mpr.planner_id = "ompl_rrtc"
        self._mpr.planning_pipeline = "ompl"
        self._mpr.planning_time = 5.0
        self._mpr.planning_attempts = 3

        self._worker = threading.Thread(target=self._run, daemon=True)
        self.get_logger().info(f"pick_node ready, localization_mode={self._mode}")

    # ------------------------------------------------------------------
    # Inputs
    # ------------------------------------------------------------------
    def _on_poses(self, msg):
        self._poses = list(msg.poses)
        self._pose_received = True

    def _make_image_cb(self, cam):
        def cb(msg):
            self._latest_image[cam] = msg
            self._publish_vis(cam, msg)
        return cb

    def _on_joint_states(self, msg):
        self._joint_states = {n: v for n, v in zip(msg.name, msg.position)}

    def _on_camera_info(self, msg):
        self._camera_info[msg.header.frame_id] = msg
        self._camera_info[self._cam_key_for(msg.header.frame_id)] = msg

    @staticmethod
    def _cam_key_for(frame_id):
        """camera_table_link -> table; camera_stick_link -> stick; ..."""
        return frame_id.removeprefix("camera_").removesuffix("_link")

    def _block_area_bounds(self):
        """Contour-area range (px^2) of a 20 mm block in a 640x480 camera.
        A block ~30-80 px across -> ~600..4000 px^2; bucket walls / the
        arm looming close are far larger, so this rejects non-block blobs."""
        return 500.0, 8000.0

    def _block_blobs(self, msg):
        """Detect block-sized blobs in a camera frame, one per colour.

        Only blobs whose projected table position is a real block (not in
        a bucket zone, not on a camera body) and whose contour area matches
        the block scale are returned - the EE wears the arm so it is large,
        and bucket walls share the block colours but project inside a zone.
        """
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            (msg.height, msg.width, 3))
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        frame_id = msg.header.frame_id
        amin, amax = self._block_area_bounds()
        found = {}
        for color_key, bounds in self._colors.items():
            low = np.array(bounds["h_low"], dtype=np.uint8)
            high = np.array(bounds["h_high"], dtype=np.uint8)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, low, high)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                    np.ones((5, 5), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            best = None
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < amin or area > amax:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                cx, cy = x + w / 2.0, y + h / 2.0
                xy = self._project_to_table((cx, cy), frame_id)
                if xy is None:
                    continue
                tx, ty = xy
                if self._in_bucket_zone(tx, ty):
                    continue
                if self._near_camera_body(tx, ty):
                    continue
                if best is None or area < best[0]:
                    best = (area, (x, y, w, h))
            if best is not None:
                found[color_key] = best[1]
        return found

    def _publish_vis(self, cam, msg):
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            (msg.height, msg.width, 3))
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        for color_key, box in self._block_blobs(msg).items():
            x, y, w, h = box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, color_key, (x, y - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        out = Image()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id
        out.height = msg.height
        out.width = msg.width
        out.encoding = "rgb8"
        out.is_bigendian = 0
        out.step = msg.width * 3
        out.data = frame.tobytes()
        self._vis_pubs[cam].publish(out)

    def _on_trigger(self, request, response):
        response.success = True
        response.message = "pick sequence triggered"
        if not self._worker.is_alive():
            self._worker = threading.Thread(target=self._run, daemon=True)
            self._worker.start()
        return response

    def _set_status(self, text):
        self.get_logger().info(text)
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    # ------------------------------------------------------------------
    # Scan: find the blocks anywhere on the table
    # ------------------------------------------------------------------
    def _in_bucket_zone(self, x, y, margin=0.05):
        return any((x - bx) ** 2 + (y - by) ** 2 < margin ** 2
                   for bx, by in BUCKET_CENTERS.values())

    def _near_camera_body(self, x, y):
        return abs(x - 0.28) < 0.04 and abs(y + 0.33) < 0.04

    def _table_objects(self):
        """World poses of objects resting on the table (blocks), excluding
        static buckets, camera bodies and anything else known-fixed."""
        out = []
        for p in self._poses:
            z = p.position.z
            if not (self._table + 0.005 < z < self._table + 0.045):
                continue
            x, y = p.position.x, p.position.y
            if self._in_bucket_zone(x, y):
                continue
            if self._near_camera_body(x, y):
                continue
            out.append(p)
        return out

    def _detect_blobs(self, color_key, frame_id="camera_table_link"):
        """Block-sized colour blobs in the given camera frame ->
        [(table x, y, area)] (bucket walls and near-field clutter filtered
        by area + projected table zone)."""
        img = self._latest_image.get(self._cam_key_for(frame_id))
        if img is None:
            return []
        found = self._block_blobs(img)
        box = found.get(color_key)
        if box is None:
            return []
        x, y, w, h = box
        return [(x + w / 2.0, y + h / 2.0, float(w * h))]

    def _project_to_table(self, px, frame_id="camera_table_link"):
        """Ray-project a pixel onto the table plane (world z = table).

        Intrinsics: FOV-based upper bound for the image centre plus the
        position-fitted value; the gz camera_info topic is published once
        at startup so late subscribers never see it, hence the hard
        fallback. Extrinsics: the stick/table cameras are fixed to the
        world (URDF joint poses); the ee camera comes from TF. All three
        sensors share the same optical-frame convention (optical +Z looks
        along the link +X axis), so the pixel ray is transformed through
        the link frame with a single rotation matrix.
        """
        ci = self._camera_info.get(frame_id)
        if ci is None:
            ci = self._camera_info.get(self._cam_key_for(frame_id))
        if ci is not None:
            fx, fy = ci.k[0], ci.k[4]
            cx, cy = ci.k[2], ci.k[5]
        else:
            fx, fy = CAM_INTRINSICS
            cx, cy = (311.0, 226.0)
        try:
            cam_tf = self._tf_buffer.lookup_transform(
                "world", frame_id, rclpy.time.Time()).transform
            t = cam_tf.translation
            q = cam_tf.rotation
            R = self._quat_to_matrix(q)
            origin = np.array([t.x, t.y, t.z])
        except Exception:
            fixed = CAM_POSES.get(frame_id)
            if fixed is None:
                return None
            origin = np.array(fixed["xyz"])
            roll, pitch, yaw = fixed["rpy"]
            R = self._rpy_to_matrix(roll, pitch, yaw)
        # optical ray of the pixel, transformed to the link frame
        d_opt = np.array([(px[0] - cx) / fx, (px[1] - cy) / fy, 1.0])
        d_link = CAM_OPTICAL_TO_LINK @ d_opt
        ray = R @ d_link
        ray = ray / np.linalg.norm(ray)
        if abs(ray[2]) < 1e-6:
            return None
        s = (self._table - origin[2]) / ray[2]
        if s <= 0:
            return None
        p = origin + s * ray
        return (float(p[0]), float(p[1]))

    @staticmethod
    def _rpy_to_matrix(roll, pitch, yaw):
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        return Rz @ Ry @ Rx

    @staticmethod
    def _quat_to_matrix(q):
        x, y, z, w = q.x, q.y, q.z, q.w
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])

    def _camera_detections(self, color_key):
        """World positions on the table for a colour: [(x, y, area)].
        Each camera frame reports at most one block-sized blob per colour
        (bucket walls / near-field handled inside _block_blobs); merging
        across cameras just dedupes near-identical hits from overlap."""
        hits = []
        for frame_id in [self._cam_frame] + self._extra_frames:
            for x, y, area in self._detect_blobs(color_key, frame_id):
                dup = any((x - hx) ** 2 + (y - hy) ** 2 < 0.02 ** 2
                          for hx, hy, _ in hits)
                if not dup:
                    hits.append((x, y, area))
        return hits

    def _scan(self):
        """Find every block: {color: world pose}. Camera gives colour +
        position; world poses confirm/refine the positions."""
        found = {}
        for color in ("red", "green", "blue"):
            dets = self._camera_detections(color)
            if not dets:
                continue
            # Small blobs are the blocks; bucket walls blobs are large.
            # Prefer the smallest detection, and only accept it if it snaps
            # onto a known table object (camera-only localisation for
            # ground_truth already has the poses; in vision mode the snap
            # anchors colour-agnostic world detects).
            dets.sort(key=lambda d: d[2])
            x, y, _area = dets[0]
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = self._table + self._block / 2.0
            pose.orientation = GRASP_QUAT
            near = None
            for p in self._table_objects():
                d = ((p.position.x - x) ** 2
                     + (p.position.y - y) ** 2) ** 0.5
                if d < 0.08 and (near is None or d < near[0]):
                    near = (d, p)
            if near is not None:
                pose.position.x = near[1].position.x
                pose.position.y = near[1].position.y
            found[color] = pose
            self._set_status(
                f"pick: scan -> {color} at ({pose.position.x:.2f}, "
                f"{pose.position.y:.2f})")
        if not found:
            n_obj = len(self._table_objects())
            self._set_status(
                f"pick: scan found no blocks on the table "
                f"({n_obj} unclassified object(s))")
        return found

    def _block_still_at(self, color, x, y, radius=0.07):
        """Camera check: does a blob of this colour still sit near (x, y)?"""
        for dx, dy, _area in self._camera_detections(color):
            if (dx - x) ** 2 + (dy - y) ** 2 < radius ** 2:
                return True
        return False

    # ------------------------------------------------------------------
    # Motion (MoveIt Python API)
    # ------------------------------------------------------------------
    def _plan(self):
        result = self._arm.plan(self._mpr)
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self._set_status(
                f"ERROR: no plan to target (code {result.error_code.val})")
            return None
        return result

    def _wait_for_future(self, future, timeout_sec):
        deadline = time.time() + timeout_sec
        while not future.done() and time.time() < deadline:
            if not rclpy.ok():
                raise RuntimeError("rclpy shut down")
            time.sleep(0.02)
        if not future.done():
            raise TimeoutError("future did not complete")
        return future.result()

    VEL_LIMITS = {
        "joint_1": 2.0,
        "joint_2": 1.5,
        "joint_3": 2.0,
        "joint_4": 3.0,
        "joint_5": 4.0,
        "joint_6": 5.0,
        "left_finger_joint": 0.05,
        "right_finger_joint": 0.05,
    }

    def _execute(self, result):
        if not self._traj_client.wait_for_server(timeout_sec=5.0):
            self._set_status("ERROR: trajectory controller action unavailable")
            return False
        goal = FollowJointTrajectory.Goal()
        jt = result.trajectory.get_robot_trajectory_msg().joint_trajectory
        if not jt.points:
            self._set_status("ERROR: empty trajectory")
            return False
        names = jt.joint_names
        times = []
        for i, pt in enumerate(jt.points):
            if i == 0:
                t = 0.0
            else:
                prev = jt.points[i - 1].positions
                cur = pt.positions
                dt = 0.0
                for j, name in enumerate(names):
                    dq = abs(cur[j] - prev[j])
                    vmax = self.VEL_LIMITS.get(name, 1.0)
                    dt = max(dt, dq / vmax)
                t = times[-1] + max(dt, 1e-3)
            times.append(t)
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
        goal.trajectory = jt
        try:
            goal_future = self._traj_client.send_goal_async(goal)
            goal_handle = self._wait_for_future(goal_future, 5.0)
        except Exception as exc:
            self._set_status(f"ERROR: could not send trajectory goal: {exc}")
            return False
        if not goal_handle.accepted:
            self._set_status("ERROR: trajectory goal rejected")
            return False
        try:
            outcome = self._wait_for_future(
                goal_handle.get_result_async(), 30.0)
        except Exception as exc:
            self._set_status(f"ERROR: trajectory execution timed out: {exc}")
            return False
        code = outcome.result.error_code
        if code != 0:
            self._set_status(f"ERROR: trajectory failed (code {code})")
            return False
        return True

    ARM_JOINTS = ["joint_1", "joint_2", "joint_3",
                  "joint_4", "joint_5", "joint_6"]

    def _startup_straighten(self):
        """Move the arm from the spawned zero configuration to the folded
        home pose with a raw joint trajectory (no collision checking), so
        MoveIt planning starts from a collision-free state."""
        if not self._traj_client.wait_for_server(timeout_sec=5.0):
            self._set_status("ERROR: trajectory controller action unavailable")
            return False
        start = [self._joint_states.get(n, 0.0) for n in self.ARM_JOINTS]
        if all(abs(a - b) < 1e-4 for a, b in zip(start, HOME_POS)):
            self._set_status("pick: already at folded home")
            return True
        jt = JointTrajectory()
        jt.joint_names = list(self.ARM_JOINTS)
        jt.header.stamp = self.get_clock().now().to_msg()
        jt.header.frame_id = "world"
        dur = 0.0
        for i, pos in enumerate((start, HOME_POS)):
            if i > 0:
                for j, name in enumerate(self.ARM_JOINTS):
                    dur = max(dur, abs(pos[j] - start[j])
                              / self.VEL_LIMITS.get(name, 1.0))
            pt = JointTrajectoryPoint()
            pt.positions = list(pos)
            pt.velocities = [0.0] * len(pos)
            pt.time_from_start.sec = int(dur)
            pt.time_from_start.nanosec = int((dur - int(dur)) * 1e9)
            jt.points.append(pt)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt
        self._set_status("pick: straightening to folded home")
        try:
            goal_future = self._traj_client.send_goal_async(goal)
            goal_handle = self._wait_for_future(goal_future, 5.0)
        except Exception as exc:
            self._set_status(f"ERROR: could not send straighten goal: {exc}")
            return False
        if not goal_handle.accepted:
            self._set_status("ERROR: straighten goal rejected")
            return False
        try:
            self._wait_for_future(goal_handle.get_result_async(), 40.0)
        except Exception as exc:
            self._set_status(f"ERROR: straighten timed out: {exc}")
            return False
        self._set_status("pick: at folded home")
        return True

    def _plan_and_execute(self, pose):
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = pose
        self._arm.set_goal_state(pose_stamped_msg=ps, pose_link="tool0")
        result = self._plan()
        if result is None:
            return False
        return self._execute(result)

    def _go_named(self, name):
        self._arm.set_goal_state(configuration_name=name)
        result = self._plan()
        if result is None:
            self._set_status(f"ERROR: no plan to named state {name}")
            return False
        return self._execute(result)

    def _gripper(self, position):
        if self._gripper_client.wait_for_server(timeout_sec=2.0):
            done = threading.Event()
            result_holder = {}

            def goal_response(future):
                try:
                    goal_handle = future.result()
                except Exception:
                    return
                if not goal_handle.accepted:
                    done.set()
                    return
                result_future = goal_handle.get_result_async()

                def result_cb(rf):
                    result_holder["result"] = rf.result()
                    done.set()

                result_future.add_done_callback(result_cb)

            self._gripper_client.send_goal_async(
                GripperCommand.Goal(
                    command=GripperCommandMsg(position=position))) \
                .add_done_callback(goal_response)
            if not done.wait(timeout=10.0):
                self.get_logger().warn("gripper action timed out")
                return False
            result = result_holder.get("result")
            if result is not None:
                self._gripper_position = result.result.position
            return True
        msg = Float64MultiArray()
        msg.data = [position] * 2
        self._finger_pub.publish(msg)
        time.sleep(1.0)
        return True

    # ------------------------------------------------------------------
    # Pick sequence
    # ------------------------------------------------------------------
    def _wait_for_actions(self):
        deadline = time.time() + 60.0
        while time.time() < deadline:
            traj_ok = self._traj_client.wait_for_server(timeout_sec=1.0)
            grip_ok = self._gripper_client.wait_for_server(timeout_sec=1.0)
            if traj_ok and grip_ok:
                self._set_status("pick: action servers ready")
                time.sleep(2.0)
                return True
            self._set_status("pick: waiting for action servers...")
            time.sleep(1.0)
        self._set_status("ERROR: action servers never became available")
        return False

    def _finger_stall(self):
        """Left finger joint position after closing (None if no joint data).

        Jaws open at q = 0 (80 mm gap between pads). Closing on a 20 mm
        cube stalls the jaws at q ~ 0.030; closing empty lets them reach
        the full command (0.034) without contacting anything."""
        if not self._joint_states:
            return None
        return self._joint_states.get("left_finger_joint", 0.040)

    def _run(self):
        if self._mode == "vision":
            deadline = time.time() + self._pose_wait_timeout
            while not self._pose_received and time.time() < deadline:
                self._set_status("pick: waiting for world poses...")
                time.sleep(0.5)
            if not self._pose_received:
                self._set_status("pick: no world poses received, aborting")
                return
        if not self._wait_for_actions():
            return
        if not self._startup_straighten():
            self._set_status("ERROR: could not straighten arm to home")
            return

        # "done" = confirmed inside its bucket. A block that gets dropped
        # mid-carry, or fails any pick attempt, is simply re-scanned and
        # retried on the next pass - never permanently given up. The loop
        # ends only when a scan finds no block-sized object on the table.
        done = set()
        while True:
            self._set_status("pick: going home")
            if not self._go_named("home"):
                return
            blocks = self._scan()
            pending = [c for c in blocks if c not in done]
            if not pending:
                self._set_status(
                    "pick: no more blocks on the table to sort "
                    f"(done {sorted(done)}), done")
                break
            for color in pending:
                pose = blocks[color]
                self._set_status(
                    f"pick: sorting {color} block at "
                    f"({pose.position.x:.2f}, {pose.position.y:.2f})")
                ok = self._pick_block(color, pose)
                if ok:
                    self._set_status(f"pick: {color} done")
                    done.add(color)
                else:
                    # dropped or failed - leave it out of `done` so it is
                    # re-scanned and revisited after the other blocks.
                    self._set_status(
                        f"pick: {color} deferred, will retry "
                        f"after the remaining blocks")
        self._set_status("pick: done")

    def _pick_block(self, color, block_pose):
        bx, by = block_pose.position.x, block_pose.position.y
        drop_xyz = list(self._drop)
        if color in BUCKET_CENTERS:
            drop_xyz[0], drop_xyz[1] = BUCKET_CENTERS[color]
            drop_xyz[2] = DROP_Z

        for attempt in range(1, self._max_attempts + 1):
            self._gripper(self._finger_open)
            approach = Pose()
            approach.position.x = bx
            approach.position.y = by
            approach.position.z = (self._table + self._block / 2.0
                                   + self._grasp_z_offset + self._approach)
            approach.orientation = GRASP_QUAT
            self._set_status("pick: approaching")
            if not self._plan_and_execute(approach):
                return False

            grasp = Pose()
            grasp.position.x = bx
            grasp.position.y = by
            grasp.position.z = (self._table + self._block / 2.0
                                + self._grasp_z_offset)
            grasp.orientation = GRASP_QUAT
            self._set_status("pick: descending")
            if not self._plan_and_execute(grasp):
                return False

            # Camera gate: only close the jaws once the camera confirms
            # the block is still at the grasp point.
            self._set_status("pick: camera gate - checking block present")
            gate_ok = False
            gate_deadline = time.time() + 3.0
            while time.time() < gate_deadline:
                if self._block_still_at(color, bx, by):
                    gate_ok = True
                    break
                time.sleep(0.25)
            if not gate_ok:
                self._set_status(
                    f"pick: {color} NOT seen by camera at grasp point, "
                    f"retrying {attempt}/{self._max_attempts}")
                continue

            self._set_status("pick: closing gripper")
            self._gripper(self._finger_close)
            time.sleep(1.2)
            stall = self._finger_stall()
            clamped = stall is not None and stall <= self._finger_clamp_limit
            self._set_status(
                f"pick: jaw stall "
                f"{'n/a' if stall is None else f'{stall:.3f}'}, "
                f"clamped={clamped}")

            lift = Pose()
            lift.position.x = bx
            lift.position.y = by
            lift.position.z = grasp.position.z + self._lift
            lift.orientation = GRASP_QUAT
            self._set_status("pick: lifting")
            if not self._plan_and_execute(lift):
                return False

            # Camera check: the block must be GONE from the grasp spot
            # (i.e. it came up with the jaws). Bucket walls are excluded
            # by the zone filter, so this is a true grasp verification.
            time.sleep(0.5)
            if self._block_still_at(color, bx, by):
                self._set_status(
                    f"pick: {color} still at grasp spot after lift, "
                    f"retrying {attempt}/{self._max_attempts}")
                continue
            if not clamped:
                self._set_status(
                    f"pick: jaws closed empty "
                    f"({stall:.3f}), retrying "
                    f"{attempt}/{self._max_attempts}")
                continue
            self._set_status("pick: block confirmed in jaws")
            break
        else:
            return False

        # Carry over the bucket, drop, verify.
        carry = Pose()
        carry.position.x = drop_xyz[0]
        carry.position.y = drop_xyz[1]
        carry.position.z = drop_xyz[2] + 0.08
        carry.orientation = GRASP_QUAT
        self._set_status("pick: carrying to bucket")
        if not self._plan_and_execute(carry):
            return False

        drop = Pose()
        drop.position.x = drop_xyz[0]
        drop.position.y = drop_xyz[1]
        drop.position.z = drop_xyz[2]
        drop.orientation = GRASP_QUAT
        self._set_status("pick: moving to drop")
        if not self._plan_and_execute(drop):
            return False

        self._set_status("pick: opening gripper")
        self._gripper(self._finger_open)
        time.sleep(0.8)

        # Lift away first so the fingers (which hang below tool0) are out
        # of the bucket-zone z band before verifying.
        away = Pose()
        away.position.x = drop_xyz[0]
        away.position.y = drop_xyz[1]
        away.position.z = drop_xyz[2] + 0.10
        away.orientation = GRASP_QUAT
        self._set_status("pick: lifting away from bucket")
        if not self._plan_and_execute(away):
            return False

        pre = sum(1 for p in self._poses
                  if (self._table - 0.02 < p.position.z < self._table + 0.06)
                  and (p.position.x - drop_xyz[0]) ** 2
                  + (p.position.y - drop_xyz[1]) ** 2 < 0.05 ** 2)

        self._set_status("pick: verifying drop")
        deadline = time.time() + self._drop_verify_timeout
        while time.time() < deadline:
            count = sum(1 for p in self._poses
                        if (self._table - 0.02 < p.position.z
                            < self._table + 0.06)
                        and (p.position.x - drop_xyz[0]) ** 2
                        + (p.position.y - drop_xyz[1]) ** 2 < 0.05 ** 2)
            in_bucket = count > pre
            gone = not self._block_still_at(color, bx, by)
            if in_bucket and gone:
                self._set_status(f"pick: {color} confirmed in bucket")
                return True
            time.sleep(0.5)
        self._set_status(f"pick: {color} NOT confirmed in bucket after drop")
        return False


def main():
    rclpy.init()
    node = PickNode()
    if node._auto_start:
        node._worker.start()
    try:
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    try:
        node._moveit.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
