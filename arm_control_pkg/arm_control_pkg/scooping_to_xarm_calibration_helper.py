#!/usr/bin/env python3
import json
import math
import threading
import time
from typing import List, Tuple

import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from std_srvs.srv import Trigger
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)

from feeding_msgs.action import MoveToPose
from xarm.wrapper import XArmAPI


def rpy_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rz @ ry @ rx


def rot_to_rpy(rot: np.ndarray) -> Tuple[float, float, float]:
    sy = math.sqrt(rot[0, 0] * rot[0, 0] + rot[1, 0] * rot[1, 0])
    singular = sy < 1e-6

    if not singular:
        roll = math.atan2(rot[2, 1], rot[2, 2])
        pitch = math.atan2(-rot[2, 0], sy)
        yaw = math.atan2(rot[1, 0], rot[0, 0])
    else:
        roll = math.atan2(-rot[1, 2], rot[1, 1])
        pitch = math.atan2(-rot[2, 0], sy)
        yaw = 0.0

    return roll, pitch, yaw


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    nrm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if nrm == 0.0:
        raise ValueError("Quaternion norm is zero.")

    qx /= nrm
    qy /= nrm
    qz /= nrm
    qw /= nrm

    return np.array(
        [
            [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
            [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
            [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )


class ScoopingToXarmCalibrationHelper(Node):
    """Manual-confirm calibration helper for AprilTag-to-xArm gripper offset mapping.

    Workflow:
    1) Move Lite6 to named perception pose.
    2) Read AprilTag pose in xArm base frame from TF.
    3) Open xArm gripper and wait for manual arm move.
    4) On confirm service: capture xArm current pose, close gripper, save sample pair.
    5) Solve service computes fixed transform tag_frame -> gripper_target.
    """

    def __init__(self):
        super().__init__("scooping_to_xarm_calibration_helper")

        # Lite6 interfaces
        self.declare_parameter("move_action_name", "move_to_pose")
        self.declare_parameter("perception_pose_name", "perception")
        self.declare_parameter("move_lite6_before_sample", False)
        self.declare_parameter("motion_type", 0)
        self.declare_parameter("action_timeout_sec", 120.0)

        # AprilTag sampling
        self.declare_parameter("apriltag_target_id", 0)
        self.declare_parameter("tag_frame_templates", ["tag36h11:{id}", "tag{id}", "tag_{id}"])
        self.declare_parameter("tf_timeout_sec", 0.6)
        self.declare_parameter("max_tag_age_sec", 0.8)
        self.declare_parameter("strict_tag_age_check", True)
        self.declare_parameter("request_retries", 2)

        # Target frame names
        self.declare_parameter("xarm_base_frame", "xarm2_base")

        # xArm settings
        self.declare_parameter("xarm_robot_ip", "192.168.1.201")
        self.declare_parameter("xarm_gripper_open_pos", 850.0)
        self.declare_parameter("xarm_gripper_close_pos", 120.0)
        self.declare_parameter("xarm_gripper_speed", 2000.0)

        # Service names
        self.declare_parameter("run_once_service", "arm2/helper/run_once")
        self.declare_parameter("open_gripper_service", "arm2/helper/open_gripper")
        self.declare_parameter("confirm_manual_pose_service", "arm2/helper/confirm_manual_pose")
        self.declare_parameter("solve_offset_service", "arm2/helper/solve_offset")
        self.declare_parameter("reset_service", "arm2/helper/reset")

        self.declare_parameter("min_samples", 4)

        self.move_action_name = self.get_parameter("move_action_name").value
        self.perception_pose_name = self.get_parameter("perception_pose_name").value
        self.move_lite6_before_sample = bool(self.get_parameter("move_lite6_before_sample").value)
        self.motion_type = int(self.get_parameter("motion_type").value)
        self.action_timeout_sec = float(self.get_parameter("action_timeout_sec").value)

        self.apriltag_target_id = int(self.get_parameter("apriltag_target_id").value)
        self.tag_frame_templates = [str(v) for v in self.get_parameter("tag_frame_templates").value]
        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self.max_tag_age_sec = float(self.get_parameter("max_tag_age_sec").value)
        self.strict_tag_age_check = bool(self.get_parameter("strict_tag_age_check").value)
        self.request_retries = int(self.get_parameter("request_retries").value)

        self.xarm_base_frame = self.get_parameter("xarm_base_frame").value

        self.xarm_robot_ip = self.get_parameter("xarm_robot_ip").value
        self.xarm_gripper_open_pos = float(self.get_parameter("xarm_gripper_open_pos").value)
        self.xarm_gripper_close_pos = float(self.get_parameter("xarm_gripper_close_pos").value)
        self.xarm_gripper_speed = float(self.get_parameter("xarm_gripper_speed").value)

        self.min_samples = int(self.get_parameter("min_samples").value)

        if not self.tag_frame_templates:
            raise ValueError("tag_frame_templates must contain at least one template")

        self._lock = threading.Lock()
        self._callback_group = ReentrantCallbackGroup()

        self._move_client = ActionClient(
            self,
            MoveToPose,
            self.move_action_name,
            callback_group=self._callback_group,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._run_srv = self.create_service(
            Trigger,
            self.get_parameter("run_once_service").value,
            self._run_once_cb,
            callback_group=self._callback_group,
        )
        self._open_srv = self.create_service(
            Trigger,
            self.get_parameter("open_gripper_service").value,
            self._open_gripper_cb,
            callback_group=self._callback_group,
        )
        self._confirm_srv = self.create_service(
            Trigger,
            self.get_parameter("confirm_manual_pose_service").value,
            self._confirm_manual_pose_cb,
            callback_group=self._callback_group,
        )
        self._solve_srv = self.create_service(
            Trigger,
            self.get_parameter("solve_offset_service").value,
            self._solve_offset_cb,
            callback_group=self._callback_group,
        )
        self._reset_srv = self.create_service(
            Trigger,
            self.get_parameter("reset_service").value,
            self._reset_cb,
            callback_group=self._callback_group,
        )

        # Pending sample captured by run_once and finalized by confirm
        self._pending_tag_position = None  # np.array([x,y,z]) in xArm base frame
        self._pending_tag_rotation = None  # np.array(3x3), tag orientation in xArm base
        self._pending_tag_frame = None
        self._pending_tag_age_sec = None

        # Dataset
        self._tag_positions = []        # Nx3, tag position in xArm base
        self._tag_rotations = []        # list of 3x3, tag orientation in xArm base
        self._target_positions = []     # Nx3, xArm base (meters)
        self._target_rotations = []     # list of 3x3, gripper orientation in xArm base
        self._target_rpy = []           # Nx3, gripper RPY in xArm base
        self._tag_frame_history = []    # N strings
        self._tag_age_history = []      # N floats

        self.arm = None
        self._connect_xarm()

        self.get_logger().info("AprilTag-to-xArm calibration helper started.")
        self.get_logger().info(
            "Workflow: run_once -> manually move xArm -> confirm_manual_pose (repeat) -> solve_offset"
        )

    def _spin_until_done(self, future, timeout_sec: float) -> bool:
        elapsed = 0.0
        step = 0.1
        while rclpy.ok() and not future.done() and elapsed < timeout_sec:
            time.sleep(step)
            elapsed += step
        return future.done()

    def _connect_xarm(self):
        self.arm = XArmAPI(port=self.xarm_robot_ip, is_radian=True)
        self.arm.motion_enable(enable=True)
        self.arm.clean_error()
        self.arm.set_mode(0)
        self.arm.set_state(0)

        ret_enable = self.arm.set_gripper_enable(True)
        ret_mode = self.arm.set_gripper_mode(0)
        ret_speed = self.arm.set_gripper_speed(self.xarm_gripper_speed)
        if ret_enable != 0 or ret_mode != 0 or ret_speed != 0:
            self.get_logger().warn(
                f"gripper setup returns: enable={ret_enable}, mode={ret_mode}, speed={ret_speed}"
            )

    def _open_gripper(self) -> Tuple[bool, str]:
        ret = self.arm.set_gripper_position(
            float(self.xarm_gripper_open_pos),
            wait=True,
            speed=self.xarm_gripper_speed,
            auto_enable=True,
        )
        if ret != 0:
            return False, f"Failed to open gripper (code={ret})"
        return True, "Opened gripper"

    def _close_gripper(self) -> Tuple[bool, str]:
        ret = self.arm.set_gripper_position(
            float(self.xarm_gripper_close_pos),
            wait=True,
            speed=self.xarm_gripper_speed,
            auto_enable=True,
        )
        if ret != 0:
            return False, f"Failed to close gripper (code={ret})"
        return True, "Closed gripper"

    def _move_lite6_to_perception(self) -> Tuple[bool, str]:
        if not self._move_client.wait_for_server(timeout_sec=self.action_timeout_sec):
            return False, f"Action server '{self.move_action_name}' unavailable"

        goal = MoveToPose.Goal()
        goal.named_pose = True
        goal.pose_name = self.perception_pose_name
        goal.speed = 0.0
        goal.mvacc = 0.0
        goal.motion_type = self.motion_type

        send_future = self._move_client.send_goal_async(goal)
        if not self._spin_until_done(send_future, self.action_timeout_sec):
            return False, "Timed out waiting for goal acceptance"

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, "MoveToPose goal rejected"

        result_future = goal_handle.get_result_async()
        if not self._spin_until_done(result_future, self.action_timeout_sec):
            return False, "Timed out waiting for MoveToPose result"

        wrapped = result_future.result()
        if wrapped is None or wrapped.result is None:
            return False, "Empty MoveToPose result"

        result = wrapped.result
        if not result.success:
            return False, f"Failed moving Lite6 to '{self.perception_pose_name}': {result.message}"

        return True, f"Lite6 moved to '{self.perception_pose_name}'"

    def _read_xarm_pose(self):
        ret, pose = self.arm.get_position(is_radian=True)
        if ret != 0 or pose is None or len(pose) < 6:
            return None, f"Failed to read xArm pose (code={ret})"

        pos_m = np.array([float(pose[0]), float(pose[1]), float(pose[2])], dtype=np.float64) / 1000.0
        rpy = np.array([float(pose[3]), float(pose[4]), float(pose[5])], dtype=np.float64)
        rot = rpy_to_rot(float(rpy[0]), float(rpy[1]), float(rpy[2]))
        return (pos_m, rot, rpy), None

    @staticmethod
    def _average_rotations(rotations: List[np.ndarray]) -> np.ndarray:
        accum = np.zeros((3, 3), dtype=np.float64)
        for rot in rotations:
            accum += rot

        u_mat, _, vt_mat = np.linalg.svd(accum)
        rot_avg = u_mat @ vt_mat
        if np.linalg.det(rot_avg) < 0:
            u_mat[:, 2] *= -1.0
            rot_avg = u_mat @ vt_mat
        return rot_avg

    @staticmethod
    def _angular_error_deg(rot_pred: np.ndarray, rot_obs: np.ndarray) -> float:
        rot_err = rot_pred.T @ rot_obs
        trace_val = max(-1.0, min(3.0, float(np.trace(rot_err))))
        cos_theta = max(-1.0, min(1.0, (trace_val - 1.0) * 0.5))
        theta = math.acos(cos_theta)
        return float(theta * 180.0 / math.pi)

    @staticmethod
    def _time_to_sec(stamp) -> float:
        return float(stamp.sec) + (float(stamp.nanosec) * 1e-9)

    def _tag_frame_candidates(self) -> List[str]:
        candidates = []
        for template in self.tag_frame_templates:
            try:
                frame = template.format(id=self.apriltag_target_id)
            except Exception:
                continue
            frame = str(frame).strip()
            if frame and frame not in candidates:
                candidates.append(frame)
        return candidates

    def _lookup_apriltag_pose(self):
        candidates = self._tag_frame_candidates()
        if not candidates:
            return None, None, None, "No valid tag frame candidates from tag_frame_templates"

        last_error = None
        for candidate in candidates:
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.xarm_base_frame,
                    candidate,
                    Time(),
                    timeout=Duration(seconds=self.tf_timeout_sec),
                )

                tr = tf_msg.transform.translation
                rot = tf_msg.transform.rotation
                pos = np.array([float(tr.x), float(tr.y), float(tr.z)], dtype=np.float64)
                rot_mat = quat_to_rot(float(rot.x), float(rot.y), float(rot.z), float(rot.w))

                now_sec = self.get_clock().now().nanoseconds * 1e-9
                stamp_sec = self._time_to_sec(tf_msg.header.stamp)
                if stamp_sec > 0.0:
                    age_sec = max(0.0, now_sec - stamp_sec)
                else:
                    age_sec = self.max_tag_age_sec + 1.0

                if self.strict_tag_age_check and age_sec > self.max_tag_age_sec:
                    return None, None, None, (
                        f"Tag pose stale for frame '{candidate}': age={age_sec:.3f}s "
                        f"(limit={self.max_tag_age_sec:.3f}s)"
                    )

                return pos, rot_mat, candidate, age_sec
            except (LookupException, ConnectivityException, ExtrapolationException) as exc:
                last_error = exc

        return None, None, None, (
            f"TF lookup failed for tag candidates {candidates} -> {self.xarm_base_frame}; "
            f"last error: {last_error}"
        )

    def _run_once_cb(self, _request, response):
        with self._lock:
            if self.move_lite6_before_sample:
                ok, msg = self._move_lite6_to_perception()
                if not ok:
                    response.success = False
                    response.message = msg
                    return response

            tag_pos = None
            tag_rot = None
            tag_frame = None
            last_error = None
            for attempt in range(1, self.request_retries + 2):
                tag_pos, tag_rot, tag_frame, err = self._lookup_apriltag_pose()
                if tag_pos is not None:
                    age_sec = float(err)
                    break
                last_error = str(err)
                self.get_logger().warn(
                    f"AprilTag lookup attempt {attempt}/{self.request_retries + 1} failed: {last_error}"
                )

            if tag_pos is None:
                response.success = False
                response.message = f"Failed to get AprilTag pose: {last_error}"
                return response

            self._pending_tag_position = tag_pos.copy()
            self._pending_tag_rotation = tag_rot.copy()
            self._pending_tag_frame = str(tag_frame)
            self._pending_tag_age_sec = float(age_sec)

            ok, grip_msg = self._open_gripper()
            if not ok:
                response.success = False
                response.message = grip_msg
                return response

            tf_publishers = self.count_publishers("/tf")

            response.success = True
            response.message = (
                f"Prepared sample from tag_frame={self._pending_tag_frame}; "
                f"tag_pos_in_{self.xarm_base_frame}=({tag_pos[0]:.4f}, {tag_pos[1]:.4f}, {tag_pos[2]:.4f}) m; "
                f"tag_age={self._pending_tag_age_sec:.3f}s; "
                f"gripper_opened. Manually move xArm to expected grasp pose, then call confirm service. "
                f"TF publishers check: /tf={tf_publishers}."
            )
            self.get_logger().info(response.message)
            return response

    def _open_gripper_cb(self, _request, response):
        ok, msg = self._open_gripper()
        response.success = ok
        response.message = msg
        return response

    def _confirm_manual_pose_cb(self, _request, response):
        with self._lock:
            if self._pending_tag_position is None or self._pending_tag_rotation is None:
                response.success = False
                response.message = "No pending sample. Call run_once first."
                return response

            pose, err = self._read_xarm_pose()
            if pose is None:
                response.success = False
                response.message = err
                return response

            pos_m, rot_m, rpy = pose

            ok, grip_msg = self._close_gripper()
            if not ok:
                response.success = False
                response.message = grip_msg
                return response

            self._tag_positions.append(self._pending_tag_position.copy())
            self._tag_rotations.append(self._pending_tag_rotation.copy())
            self._tag_frame_history.append(str(self._pending_tag_frame))
            self._tag_age_history.append(float(self._pending_tag_age_sec))

            self._target_positions.append(pos_m.copy())
            self._target_rotations.append(rot_m.copy())
            self._target_rpy.append(rpy.copy())

            sample_count = len(self._tag_positions)

            # clear pending
            self._pending_tag_position = None
            self._pending_tag_rotation = None
            self._pending_tag_frame = None
            self._pending_tag_age_sec = None

            response.success = True
            response.message = (
                f"Captured sample #{sample_count}: gripper_pos=({pos_m[0]:.4f}, {pos_m[1]:.4f}, {pos_m[2]:.4f}) m, "
                f"gripper_rpy=({rpy[0]:.4f}, {rpy[1]:.4f}, {rpy[2]:.4f}) rad."
            )
            self.get_logger().info(response.message)
            return response

    def _solve_offset_cb(self, _request, response):
        with self._lock:
            n = len(self._tag_positions)
            if n < self.min_samples:
                response.success = False
                response.message = f"Need at least {self.min_samples} samples, but have {n}."
                return response

            tag_pos = np.array(self._tag_positions, dtype=np.float64)
            gripper_pos = np.array(self._target_positions, dtype=np.float64)
            rpy_samples = np.array(self._target_rpy, dtype=np.float64)

            tag_to_gripper_transforms = []
            tag_to_gripper_rotations = []

            for i in range(n):
                rot_x_t = self._tag_rotations[i]
                rot_x_g = self._target_rotations[i]

                rot_t_g = rot_x_t.T @ rot_x_g
                t_t_g = rot_x_t.T @ (gripper_pos[i] - tag_pos[i])

                tag_to_gripper_rotations.append(rot_t_g)
                tag_to_gripper_transforms.append(t_t_g)

            rot_t_g = self._average_rotations(tag_to_gripper_rotations)
            t_t_g = np.mean(np.array(tag_to_gripper_transforms, dtype=np.float64), axis=0)
            t_t_g_std = np.std(np.array(tag_to_gripper_transforms, dtype=np.float64), axis=0)

            rot_g_t = rot_t_g.T
            t_g_t = -(rot_g_t @ t_t_g)

            roll_t_g, pitch_t_g, yaw_t_g = rot_to_rpy(rot_t_g)
            roll_g_t, pitch_g_t, yaw_g_t = rot_to_rpy(rot_g_t)

            pos_errors = []
            ang_errors = []
            residual_vectors = []
            tg_rot_dev = []
            for i in range(n):
                rot_x_t = self._tag_rotations[i]
                rot_x_g = self._target_rotations[i]

                pred_pos = tag_pos[i] + (rot_x_t @ t_t_g)
                pred_rot = rot_x_t @ rot_t_g

                diff = pred_pos - gripper_pos[i]
                residual_vectors.append(diff)
                pos_errors.append(float(np.linalg.norm(diff)))
                ang_errors.append(self._angular_error_deg(pred_rot, rot_x_g))

                tg_rot_dev.append(self._angular_error_deg(rot_t_g, tag_to_gripper_rotations[i]))

            pos_errors = np.array(pos_errors, dtype=np.float64)
            ang_errors = np.array(ang_errors, dtype=np.float64)
            residual_vectors = np.array(residual_vectors, dtype=np.float64)
            tg_rot_dev = np.array(tg_rot_dev, dtype=np.float64)

            pos_rmse = math.sqrt(float(np.mean(pos_errors * pos_errors)))
            residual_mean = np.mean(residual_vectors, axis=0)
            residual_std = np.std(residual_vectors, axis=0)
            rpy_mean = np.mean(rpy_samples, axis=0)

            output = {
                "num_samples": int(n),
                "xarm_base_frame": self.xarm_base_frame,
                "apriltag_target_id": int(self.apriltag_target_id),
                "tag_frame_samples": list(sorted(set(self._tag_frame_history))),
                "transform_gripper_from_tag": {
                    "tx": float(t_t_g[0]),
                    "ty": float(t_t_g[1]),
                    "tz": float(t_t_g[2]),
                    "roll": float(roll_t_g),
                    "pitch": float(pitch_t_g),
                    "yaw": float(yaw_t_g),
                },
                "transform_tag_from_gripper": {
                    "tx": float(t_g_t[0]),
                    "ty": float(t_g_t[1]),
                    "tz": float(t_g_t[2]),
                    "roll": float(roll_g_t),
                    "pitch": float(pitch_g_t),
                    "yaw": float(yaw_g_t),
                },
                "residual_offset_mean_xyz": residual_mean.tolist(),
                "residual_offset_std_xyz": residual_std.tolist(),
                "position_rmse_m": float(pos_rmse),
                "position_error_mean_m": float(np.mean(pos_errors)),
                "position_error_max_m": float(np.max(pos_errors)),
                "angular_error_mean_deg": float(np.mean(ang_errors)),
                "angular_error_std_deg": float(np.std(ang_errors)),
                "angular_error_max_deg": float(np.max(ang_errors)),
                "tag_to_gripper_translation_std_m": t_t_g_std.tolist(),
                "tag_to_gripper_rotation_consistency_deg_mean": float(np.mean(tg_rot_dev)),
                "recommended_gripper_rpy_mean": rpy_mean.tolist(),
                "sample_tag_age_mean_sec": float(np.mean(np.array(self._tag_age_history, dtype=np.float64))),
                "sample_tag_age_max_sec": float(np.max(np.array(self._tag_age_history, dtype=np.float64))),
            }

            yaml_snippet = (
                "arm2_scooping_grasp:\n"
                "  ros__parameters:\n"
                f"    tag_to_gripper_xyz: [{t_t_g[0]:.6f}, {t_t_g[1]:.6f}, {t_t_g[2]:.6f}]\n"
                f"    tag_to_gripper_rpy: [{roll_t_g:.6f}, {pitch_t_g:.6f}, {yaw_t_g:.6f}]"
            )

            self.get_logger().info("Solved AprilTag->gripper calibration:")
            self.get_logger().info(json.dumps(output, indent=2))
            self.get_logger().info("Use the following parameters in arm2_scooping_grasp.yaml:")
            self.get_logger().info(yaml_snippet)

            response.success = True
            response.message = (
                f"Solved with {n} samples; position_rmse={pos_rmse:.6f} m; "
                f"tag->gripper t=({t_t_g[0]:.4f},{t_t_g[1]:.4f},{t_t_g[2]:.4f}); "
                f"recommended_rpy_mean=({rpy_mean[0]:.4f},{rpy_mean[1]:.4f},{rpy_mean[2]:.4f}). "
                f"JSON:\n{json.dumps(output)}"
            )
            return response

    def _reset_cb(self, _request, response):
        with self._lock:
            self._pending_tag_position = None
            self._pending_tag_rotation = None
            self._pending_tag_frame = None
            self._pending_tag_age_sec = None

            self._tag_positions = []
            self._tag_rotations = []
            self._target_positions = []
            self._target_rotations = []
            self._target_rpy = []
            self._tag_frame_history = []
            self._tag_age_history = []

        response.success = True
        response.message = "Helper state reset."
        self.get_logger().info(response.message)
        return response

    def shutdown(self):
        if self.arm is not None:
            try:
                self.arm.disconnect()
            except Exception as exc:
                self.get_logger().warn(f"Error during xArm disconnect: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = ScoopingToXarmCalibrationHelper()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down scooping_to_xarm_calibration_helper")
    finally:
        executor.shutdown()
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
