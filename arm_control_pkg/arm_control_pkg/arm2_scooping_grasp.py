#!/usr/bin/env python3
import json
import math
import threading
import time

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)

from feeding_msgs.srv import GetBowlFoodRatio
from feeding_mujoco.tb_scoop_model_optimiser import TiltOptimiser
from feeding_mujoco.tb_scoop_traj_generator import get_new_bowl_pose
# from feeding_mujoco.feeding_mujoco.tb_scoop_volume_mass_model import VolumeMassModel
from xarm.wrapper import XArmAPI


def quat_to_rpy(x: float, y: float, z: float, w: float):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def rpy_to_quat(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b

    x = aw * bx + ax * bw + ay * bz - az * by
    y = aw * by - ax * bz + ay * bw + az * bx
    z = aw * bz + ax * by - ay * bx + az * bw
    w = aw * bw - ax * bx - ay * by - az * bz
    return x, y, z, w


def quat_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return 0.0, 0.0, 0.0, 1.0
    return x / n, y / n, z / n, w / n


def rotate_vector_by_quat(q, vec):
    x, y, z, w = quat_normalize(q)
    vx, vy, vz = vec

    r00 = 1.0 - 2.0 * (y * y + z * z)
    r01 = 2.0 * (x * y - z * w)
    r02 = 2.0 * (x * z + y * w)

    r10 = 2.0 * (x * y + z * w)
    r11 = 1.0 - 2.0 * (x * x + z * z)
    r12 = 2.0 * (y * z - x * w)

    r20 = 2.0 * (x * z - y * w)
    r21 = 2.0 * (y * z + x * w)
    r22 = 1.0 - 2.0 * (x * x + y * y)

    return (
        r00 * vx + r01 * vy + r02 * vz,
        r10 * vx + r11 * vy + r12 * vz,
        r20 * vx + r21 * vy + r22 * vz,
    )


class Arm2ScoopingGrasp(Node):
    """Service-triggered grasp node for a second xArm6.

    Flow:
    1) Resolve the selected bowl to its configured AprilTag.
    2) Request the AprilTag target pose.
    3) Convert the target pose to an xArm command pose.
    4) Execute open -> approach -> close -> lift sequence.
    5) If the selected bowl maps to no tag, skip arm motion as a no-op.
    """

    def __init__(self):
        super().__init__('arm2_scooping_grasp')

        # Core connectivity
        self.declare_parameter('robot_ip', '192.168.1.201')
        self.declare_parameter('trigger_service_name', 'arm2/execute_grasp')
        self.declare_parameter('return_to_init_service_name', 'arm2/return_to_saved_initial_pose')
        self.declare_parameter('selected_bowl_idx', -1)
        self.declare_parameter('auto_execute_on_index_set', True)
        self.declare_parameter('auto_execute_done_seq', 0)
        self.declare_parameter('auto_execute_status', 'idle')
        self.declare_parameter('auto_execute_message', 'Idle.')
        self.declare_parameter('target_frame', 'xarm2_base')
        self.declare_parameter('tf_timeout_sec', 1.0)
        self.declare_parameter('service_timeout_sec', 5.0)
        self.declare_parameter('bowl_idx_to_tag_id', [-1, 1, 0])
        self.declare_parameter('tag_frame_templates', ['tag36h11:{id}', 'tag{id}', 'tag_{id}'])
        self.declare_parameter('tag_to_gripper_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('tag_to_gripper_rpy', [0.0, 0.0, 0.0])
        self.declare_parameter('apriltag_request_retries', 1)
        self.declare_parameter('apriltag_pose_max_age_sec', 0.5)
        self.declare_parameter('use_apriltag_orientation', True)

        # Motion parameters (xArm expects mm and rad)
        self.declare_parameter('default_speed', 60.0)
        self.declare_parameter('default_accel', 120.0)
        self.declare_parameter('motion_type', 0)
        self.declare_parameter('use_current_orientation', True)
        self.declare_parameter('target_orientation_rpy', [0.0, -1.571, 3.141])

        # Point offsets (meters)
        self.declare_parameter('point_offset_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('approach_z_offset', 0.08)
        self.declare_parameter('grasp_z_offset', 0.0)
        self.declare_parameter('lift_z_offset', 0.12)

        # Gripper parameters
        self.declare_parameter('gripper_open_pos', 850.0)
        self.declare_parameter('gripper_close_pos', 120.0)
        self.declare_parameter('gripper_speed', 2000.0)

        # Adaptive bowl-tilt parameters
        self.declare_parameter('get_bowl_food_ratio_service', 'food_perception/get_bowl_food_ratio')
        # Desired bite size (grams). Set by task_planner before triggering the grasp.
        self.declare_parameter('target_scoop_ml', 8.0)
        # Predetermined pose (mm, rad) from which to estimate the bowl food volume.
        # Placeholder default reuses the final tilt pose; update with a real pose later.
        self.declare_parameter(
            'volume_estimation_pose_6dof',
            [242.1, -99.3, 74.1, math.radians(-164.4), math.radians(-40.7), math.radians(-116.1)],
        )
        self.declare_parameter('minimum_food_volume_m3', 1.5e-5)

        self.declare_parameter('init_expected_volume_m3', 1.5e-5)
        # ROS parameters do not support dictionaries directly, so per-bowl
        # volume-check results are exposed as a JSON-encoded dictionary.
        self.declare_parameter('volume_check_status_by_bowl_json', '{}')
        self.declare_parameter('volume_offset_ml', 10.0)  # Optional offset to add to the volume estimate (ml)
        # Last computed optimal tilt angle (deg). Exposed for the task_planner to read.
        self.declare_parameter('last_tilt_angle_deg', 0.0)

        # Optional workspace safety limits in mm (target frame)
        self.declare_parameter('use_workspace_limits', False)
        self.declare_parameter('workspace_min_xyz', [200.0, -400.0, 80.0])
        self.declare_parameter('workspace_max_xyz', [700.0, 400.0, 500.0])

        self.robot_ip = self.get_parameter('robot_ip').value
        self.trigger_service_name = self.get_parameter('trigger_service_name').value
        self.return_to_init_service_name = self.get_parameter('return_to_init_service_name').value
        self.target_frame = self.get_parameter('target_frame').value
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)
        self.service_timeout_sec = float(self.get_parameter('service_timeout_sec').value)
        self.bowl_idx_to_tag_id = list(self.get_parameter('bowl_idx_to_tag_id').value)
        self.tag_frame_templates = [str(v) for v in self.get_parameter('tag_frame_templates').value]
        self.tag_to_gripper_xyz = [float(v) for v in self.get_parameter('tag_to_gripper_xyz').value]
        self.tag_to_gripper_rpy = [float(v) for v in self.get_parameter('tag_to_gripper_rpy').value]
        self.apriltag_request_retries = int(self.get_parameter('apriltag_request_retries').value)
        self.apriltag_pose_max_age_sec = float(self.get_parameter('apriltag_pose_max_age_sec').value)
        self.use_apriltag_orientation = bool(self.get_parameter('use_apriltag_orientation').value)

        self.default_speed = float(self.get_parameter('default_speed').value)
        self.default_accel = float(self.get_parameter('default_accel').value)
        self.motion_type = int(self.get_parameter('motion_type').value)
        self.use_current_orientation = bool(self.get_parameter('use_current_orientation').value)
        self.target_orientation_rpy = list(self.get_parameter('target_orientation_rpy').value)

        self.point_offset_xyz = list(self.get_parameter('point_offset_xyz').value)
        self.approach_z_offset = float(self.get_parameter('approach_z_offset').value)
        self.grasp_z_offset = float(self.get_parameter('grasp_z_offset').value)
        self.lift_z_offset = float(self.get_parameter('lift_z_offset').value)
        self.volume_offset_ml = float(self.get_parameter('volume_offset_ml').value)
        self.gripper_open_pos = float(self.get_parameter('gripper_open_pos').value)
        self.gripper_close_pos = float(self.get_parameter('gripper_close_pos').value)
        self.gripper_speed = float(self.get_parameter('gripper_speed').value)
        self.auto_execute_on_index_set = bool(self.get_parameter('auto_execute_on_index_set').value)
        self._auto_execute_done_seq = int(self.get_parameter('auto_execute_done_seq').value)
        self._auto_execute_status = str(self.get_parameter('auto_execute_status').value)
        self._auto_execute_message = str(self.get_parameter('auto_execute_message').value)

        self.bowl_food_ratio_service_name = self.get_parameter('get_bowl_food_ratio_service').value
        self.target_scoop_ml = float(self.get_parameter('target_scoop_ml').value)
        self.volume_estimation_pose_6dof = [
            float(v) for v in self.get_parameter('volume_estimation_pose_6dof').value
        ]
        self.minimum_food_volume_m3 = float(
            self.get_parameter('minimum_food_volume_m3').value
        )
        self.init_expected_volume_m3 = float(self.get_parameter('init_expected_volume_m3').value)

        self.use_workspace_limits = bool(self.get_parameter('use_workspace_limits').value)
        self.workspace_min_xyz = list(self.get_parameter('workspace_min_xyz').value)
        self.workspace_max_xyz = list(self.get_parameter('workspace_max_xyz').value)

        if len(self.volume_estimation_pose_6dof) != 6:
            raise ValueError('volume_estimation_pose_6dof must have exactly 6 values [x, y, z, roll, pitch, yaw].')
        if (
            not math.isfinite(self.minimum_food_volume_m3)
            or self.minimum_food_volume_m3 <= 0.0
        ):
            raise ValueError('minimum_food_volume_m3 must be finite and greater than zero.')

        if len(self.target_orientation_rpy) != 3:
            raise ValueError('target_orientation_rpy must have exactly 3 values [roll, pitch, yaw].')
        if len(self.point_offset_xyz) != 3:
            raise ValueError('point_offset_xyz must have exactly 3 values [x, y, z].')
        if len(self.workspace_min_xyz) != 3 or len(self.workspace_max_xyz) != 3:
            raise ValueError('workspace_min_xyz/workspace_max_xyz must each have exactly 3 values.')
        if len(self.bowl_idx_to_tag_id) == 0:
            raise ValueError('bowl_idx_to_tag_id must contain at least one entry.')
        if len(self.tag_frame_templates) == 0:
            raise ValueError('tag_frame_templates must contain at least one template.')
        if len(self.tag_to_gripper_xyz) != 3:
            raise ValueError('tag_to_gripper_xyz must have exactly 3 values.')
        if len(self.tag_to_gripper_rpy) != 3:
            raise ValueError('tag_to_gripper_rpy must have exactly 3 values.')
        try:
            self.bowl_idx_to_tag_id = [int(v) for v in self.bowl_idx_to_tag_id]
        except Exception as exc:
            raise ValueError(f'bowl_idx_to_tag_id must be a list of integers: {exc}') from exc

        self._grasp_lock = threading.Lock()
        self._callback_group = ReentrantCallbackGroup()
        self._pending_auto_execute = False
        self._pending_auto_idx = None
        self._saved_init_pose_by_tag = {}
        self._last_successful_selected_bowl_idx = None
        self._volume_check_status_by_bowl = {}
        self._expected_volume_m3 = {}

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._bowl_food_ratio_client = self.create_client(
            GetBowlFoodRatio,
            self.bowl_food_ratio_service_name,
            callback_group=self._callback_group,
        )

        # Adaptive tilt models: detected volume -> mass, and mass -> optimal tilt angle.
        # self._volume_mass_model = VolumeMassModel()
        self._tilt_optimiser = [TiltOptimiser(bowl_index=i) for i in range(3)]

        self._trigger_srv = self.create_service(
            Trigger,
            self.trigger_service_name,
            self.execute_grasp_callback,
            callback_group=self._callback_group,
        )
        self._return_to_init_srv = self.create_service(
            Trigger,
            self.return_to_init_service_name,
            self.move_to_saved_initial_pose_callback,
            callback_group=self._callback_group,
        )
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self._auto_exec_timer = self.create_timer(
            0.1,
            self._auto_execute_timer_cb,
            callback_group=self._callback_group,
        )

        self.arm = None
        self._connect_arm()

        self.get_logger().info(
            f"Started arm2 grasp node (robot_ip={self.robot_ip}, trigger={self.trigger_service_name}, "
            f"return_to_init={self.return_to_init_service_name}, "
            f"target_frame={self.target_frame}, bowl_idx_to_tag_id={self.bowl_idx_to_tag_id})"
            f"with initial expected volume {self.init_expected_volume_m3:.6e} m^3."
        )

    @staticmethod
    def _time_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _resolve_tag_frames(self, tag_id: int):
        candidates = []
        for template in self.tag_frame_templates:
            try:
                frame = str(template).format(id=tag_id)
            except Exception:
                continue
            frame = frame.strip()
            if frame and frame not in candidates:
                candidates.append(frame)
        return candidates

    def _lookup_apriltag_target_tf(self, target_tag_id: int):
        candidates = self._resolve_tag_frames(target_tag_id)
        if not candidates:
            return None, None, None, 'No valid tag frame candidates from tag_frame_templates.'

        last_error = None
        for tag_frame in candidates:
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    tag_frame,
                    Time(),
                    timeout=Duration(seconds=self.tf_timeout_sec),
                )

                now_sec = self.get_clock().now().nanoseconds * 1e-9
                stamp_sec = self._time_to_sec(tf_msg.header.stamp)
                age_sec = max(0.0, now_sec - stamp_sec) if stamp_sec > 0.0 else (self.apriltag_pose_max_age_sec + 1.0)

                if self.apriltag_pose_max_age_sec > 0.0 and age_sec > self.apriltag_pose_max_age_sec:
                    return None, None, None, (
                        f'AprilTag TF pose stale for {tag_frame}: age={age_sec:.3f}s '
                        f'(limit={self.apriltag_pose_max_age_sec:.3f}s).'
                    )

                tr = tf_msg.transform.translation
                rot = tf_msg.transform.rotation

                tag_q = (
                    float(rot.x),
                    float(rot.y),
                    float(rot.z),
                    float(rot.w),
                )
                off_q = rpy_to_quat(
                    float(self.tag_to_gripper_rpy[0]),
                    float(self.tag_to_gripper_rpy[1]),
                    float(self.tag_to_gripper_rpy[2]),
                )

                off_xyz_rot = rotate_vector_by_quat(
                    tag_q,
                    (
                        float(self.tag_to_gripper_xyz[0]),
                        float(self.tag_to_gripper_xyz[1]),
                        float(self.tag_to_gripper_xyz[2]),
                    ),
                )

                target_xyz = (
                    float(tr.x) + off_xyz_rot[0],
                    float(tr.y) + off_xyz_rot[1],
                    float(tr.z) + off_xyz_rot[2],
                )
                target_q = quat_normalize(quat_multiply(tag_q, off_q))

                return target_xyz, target_q, tag_frame, None
            except (LookupException, ConnectivityException, ExtrapolationException) as exc:
                last_error = exc

        return None, None, None, (
            f'No transform available from tag candidates {candidates} to {self.target_frame}. '
            f'Last error: {last_error}'
        )

    def _on_set_parameters(self, params):
        result = SetParametersResult()
        result.successful = True

        for param in params:
            if param.name == 'auto_execute_on_index_set':
                self.auto_execute_on_index_set = bool(param.value)
            elif param.name == 'target_scoop_ml':
                try:
                    target_scoop_ml = float(param.value)
                except (TypeError, ValueError):
                    result.successful = False
                    result.reason = 'target_scoop_ml must be a number.'
                    return result

                if not math.isfinite(target_scoop_ml) or target_scoop_ml <= 0.0:
                    result.successful = False
                    result.reason = 'target_scoop_ml must be finite and greater than zero.'
                    return result

                self.target_scoop_ml = target_scoop_ml
                self.get_logger().info(
                    f'Updated target_scoop_ml to {self.target_scoop_ml:.2f} ml.'
                )
            elif param.name == 'bowl_idx_to_tag_id':
                try:
                    self.bowl_idx_to_tag_id = [int(v) for v in list(param.value)]
                except Exception:
                    result.successful = False
                    result.reason = 'bowl_idx_to_tag_id must be a list of integers.'
                    return result
            elif param.name == 'selected_bowl_idx' and self.auto_execute_on_index_set:
                try:
                    idx = int(param.value)
                except Exception:
                    continue

                if idx >= 0:
                    self._pending_auto_idx = idx
                    self._pending_auto_execute = True
                    self._auto_execute_status = 'pending'
                    self._auto_execute_message = (
                        f'Pending auto execute for selected_bowl_idx={idx}.'
                    )
                    self._update_auto_execute_status_params()
            elif param.name == 'init_expected_volume_m3':
                try:
                    self.get_logger().info(f'Updating init_expected_volume_m3 to {param.value}')
                    init_expected_volume_m3 = float(param.value)
                except Exception:
                    result.successful = False
                    result.reason = 'expected_volume_m3 must be a number.'
                    return result

                self.init_expected_volume_m3 = init_expected_volume_m3
                self.get_logger().info(
                    f'Updated init_expected_volume_m3 to {self.init_expected_volume_m3:.6e} m^3.'
                )

        return result

    def _update_auto_execute_status_params(self):
        try:
            self.set_parameters([
                Parameter('auto_execute_done_seq', value=int(self._auto_execute_done_seq)),
                Parameter('auto_execute_status', value=str(self._auto_execute_status)),
                Parameter('auto_execute_message', value=str(self._auto_execute_message)),
            ])
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to update auto-execute status parameters: {exc}'
            )

    def _auto_execute_timer_cb(self):
        if not self.auto_execute_on_index_set:
            return
        if not self._pending_auto_execute:
            return

        if self._grasp_lock.locked():
            return

        idx = self._pending_auto_idx
        self._pending_auto_execute = False

        self._auto_execute_status = 'running'
        self._auto_execute_message = (
            f'Auto execute started for selected_bowl_idx={idx}.'
        )
        self._update_auto_execute_status_params()

        success, message = self._execute_grasp()
        self._auto_execute_done_seq += 1
        self._auto_execute_status = 'succeeded' if success else 'failed'
        self._auto_execute_message = message
        self._update_auto_execute_status_params()

        if success:
            self.get_logger().info(
                f'Auto-executed grasp for selected_bowl_idx={idx}: {message}'
            )
        else:
            self.get_logger().warning(
                f'Auto-execute grasp failed for selected_bowl_idx={idx}: {message}'
            )

    def _connect_arm(self):
        self.arm = XArmAPI(port=self.robot_ip, is_radian=True)
        time.sleep(0.1)

        for attempt in range(1, 6):
            ret = self.arm.motion_enable(enable=True)
            if ret == 0:
                break
            self.get_logger().warn(f"motion_enable failed (attempt {attempt}/5, code={ret})")
            time.sleep(0.1)

        self.arm.clean_error()
        self.arm.set_mode(0)
        self.arm.set_state(0)
        time.sleep(0.1)

        self._setup_gripper()

    def _setup_gripper(self):
        ret_enable = self.arm.set_gripper_enable(True)
        ret_mode = self.arm.set_gripper_mode(0)
        ret_speed = self.arm.set_gripper_speed(self.gripper_speed)

        if ret_enable != 0:
            self.get_logger().warn(f"set_gripper_enable returned {ret_enable}")
        if ret_mode != 0:
            self.get_logger().warn(f"set_gripper_mode returned {ret_mode}")
        if ret_speed != 0:
            self.get_logger().warn(f"set_gripper_speed returned {ret_speed}")

    def _call_get_bowl_food_ratio(self):
        """Query GetBowlFoodRatio and return (detected_volume_m3, error_message)."""
        if not self._bowl_food_ratio_client.wait_for_service(timeout_sec=1.0):
            return None, 'GetBowlFoodRatio service is unavailable.'

        req = GetBowlFoodRatio.Request()
        future = self._bowl_food_ratio_client.call_async(req)

        deadline = time.time() + self.service_timeout_sec
        while rclpy.ok() and not future.done() and time.time() < deadline:
            time.sleep(0.02)

        if not future.done():
            return None, f'GetBowlFoodRatio call timed out after {self.service_timeout_sec:.1f}s.'

        if future.exception() is not None:
            return None, f'GetBowlFoodRatio call failed: {future.exception()}'

        resp = future.result()
        if resp is None:
            return None, 'GetBowlFoodRatio returned no response.'

        if not resp.success:
            return None, 'GetBowlFoodRatio returned success=false.'

        if not bool(resp.reference_used):
            return None, (
                'Empty-bowl reference is missing or stale; detected food volume is invalid. '
                'Capture an empty-bowl reference before running an adaptive tilt scoop.'
            )

        volume_m3 = float(resp.reference_food_volume_m3)
        if volume_m3 <= 0.0:
            return None, f'Detected food volume is non-positive ({volume_m3:.3e} m^3).'

        return volume_m3, None

    def _update_last_tilt_angle_param(self, angle_deg):
        try:
            self.set_parameters([
                Parameter('last_tilt_angle_deg', value=float(angle_deg)),
            ])
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to update last_tilt_angle_deg parameter: {exc}'
            )

    def _update_volume_check_status(self, selected_bowl_idx, volume_m3, completed):
        """Store and publish the latest volume-check result for one bowl."""
        bowl_key = int(selected_bowl_idx)
        self._volume_check_status_by_bowl[bowl_key] = {
            'last_detected_food_volume_m3': float(volume_m3),
            'last_volume_check_completed': bool(completed),
        }

        try:
            self.set_parameters([
                Parameter(
                    'volume_check_status_by_bowl_json',
                    value=json.dumps(self._volume_check_status_by_bowl, sort_keys=True),
                ),
            ])
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to update volume-check status parameters: {exc}'
            )

    def _resolve_selected_bowl_tag(self):
        try:
            idx = int(self.get_parameter('selected_bowl_idx').value)
        except Exception as exc:
            return None, None, f'Unable to read selected_bowl_idx: {exc}'

        if idx < 0:
            return None, None, (
                f'selected_bowl_idx is {idx}. Manager must set selected_bowl_idx before calling trigger.'
            )

        if idx >= len(self.bowl_idx_to_tag_id):
            return None, None, (
                f'selected_bowl_idx out of bounds for bowl_idx_to_tag_id: '
                f'idx={idx}, mapping_len={len(self.bowl_idx_to_tag_id)}.'
            )

        try:
            tag_id = int(self.bowl_idx_to_tag_id[idx])
        except Exception as exc:
            return None, None, f'Invalid tag mapping at index {idx}: {exc}'

        return idx, tag_id, None

    def _get_orientation(self):
        if self.use_current_orientation:
            try:
                ret, pose = self.arm.get_position(is_radian=True)
                if ret == 0 and pose is not None and len(pose) >= 6:
                    return float(pose[3]), float(pose[4]), float(pose[5])
                self.get_logger().warn(
                    f'get_position failed (code={ret}); using configured target_orientation_rpy.'
                )
            except Exception as exc:
                self.get_logger().warn(
                    f'Failed to read current orientation ({exc}); using configured target_orientation_rpy.'
                )

        return (
            float(self.target_orientation_rpy[0]),
            float(self.target_orientation_rpy[1]),
            float(self.target_orientation_rpy[2]),
        )

    def _move_to_mm(self, x_mm, y_mm, z_mm, roll, pitch, yaw):
        self.arm.set_state(0)
        ret = self.arm.set_position(
            x=float(x_mm),
            y=float(y_mm),
            z=float(z_mm),
            roll=float(roll),
            pitch=float(pitch),
            yaw=float(yaw),
            speed=self.default_speed,
            mvacc=self.default_accel,
            wait=True,
            motion_type=self.motion_type,
        )
        return ret

    def _vibrate_arm(self):
        amplitude = 2.0  # mm
        frequency = 10.0  # Hz (cycles per second)
        duration = 3.0   # seconds to vibrate

        # Get the current starting position
        code, current_pos = self.arm.get_position()

        # Switch to Servo Control Mode (Mode 1) for real-time streaming
        self.arm.set_mode(1)
        self.arm.set_state(0)
        time.sleep(0.1) # Brief pause to let the mode switch register

        start_time = time.time()

        try:
            while time.time() - start_time < duration:
                t = time.time() - start_time
                
                # Calculate the Z-axis offset using a sine wave
                # Sine wave formula: offset = Amplitude * sin(2 * pi * Frequency * time)
                z_offset = amplitude * math.sin(2 * math.pi * frequency * t)
                
                # Create the new target pose based on the starting position
                target_pos = current_pos.copy()
                target_pos[2] += z_offset # Applying the vibration to the Z axis
                
                # Stream the target position. 
                # is_radian=False depending on your setup, wait=False is crucial for smooth streaming
                self.arm.set_servo_cartesian(target_pos, wait=False)
                
                # Sleep briefly to match the arm's expected real-time communication frequency (~100Hz)
                time.sleep(0.01) 

        finally:
            # Always reset the arm back to normal position mode (Mode 0) when done
            self.arm.set_mode(0)
            self.arm.set_state(0)

        return 0  # Return success code    
    
    def _set_gripper_position(self, pos):
        ret = self.arm.set_gripper_position(
            float(pos),
            wait=True,
            speed=self.gripper_speed,
            auto_enable=True,
        )
        return ret

    def _within_workspace(self, x_mm, y_mm, z_mm):
        if not self.use_workspace_limits:
            return True

        min_x, min_y, min_z = [float(v) for v in self.workspace_min_xyz]
        max_x, max_y, max_z = [float(v) for v in self.workspace_max_xyz]

        return (
            min_x <= x_mm <= max_x
            and min_y <= y_mm <= max_y
            and min_z <= z_mm <= max_z
        )

    @staticmethod
    def _extract_xarm_value(resp, default_code=0):
        if isinstance(resp, tuple):
            if len(resp) >= 2:
                return int(resp[0]), resp[1]
            if len(resp) == 1:
                return default_code, resp[0]
        return default_code, resp

    def _is_arm_motion_active(self):
        # Prefer get_is_moving when available; fall back to controller state.
        try:
            code, moving = self._extract_xarm_value(self.arm.get_is_moving())
            if code == 0:
                return bool(moving), None
        except Exception:
            pass

        try:
            code, state = self._extract_xarm_value(self.arm.get_state())
            if code == 0:
                # xArm state=1 indicates moving.
                return int(state) == 1, None
            return None, f'get_state returned code={code}'
        except Exception as exc:
            return None, f'Unable to query arm motion state: {exc}'

    def _wait_for_arm_idle(self, timeout_sec=15.0, poll_sec=0.05):
        deadline = time.time() + float(timeout_sec)
        last_error = None
        consecutive_idle = 0

        while rclpy.ok() and time.time() < deadline:
            moving, err = self._is_arm_motion_active()

            if moving is None:
                return False, err or 'Arm motion status is unavailable.'

            if moving:
                consecutive_idle = 0
            else:
                consecutive_idle += 1
                if consecutive_idle >= 3:
                    return True, None

            last_error = err
            time.sleep(float(poll_sec))

        return False, (
            f'Arm did not become idle within {float(timeout_sec):.1f}s '
            f'(last_error={last_error}).'
        )

    def _move_to_saved_initial_pose(self):
        selected_idx, resolved_tag_id, map_error = self._resolve_selected_bowl_tag()
        if map_error is not None:
            return False, map_error

        if resolved_tag_id < 0:
            return False, (
                f'Cannot return to saved pose for selected_bowl_idx={selected_idx}: '
                f'mapped tag_id={resolved_tag_id} (no-tag bowl).'
            )

        saved_pose = self._saved_init_pose_by_tag.get(resolved_tag_id)
        if saved_pose is None:
            return False, (
                f'No saved initial pose for tag_id={resolved_tag_id}. '
                'Run a successful grasp for this tag first.'
            )

        x_mm, y_mm, z_mm, roll, pitch, yaw = saved_pose
        if not self._within_workspace(x_mm, y_mm, z_mm):
            return False, (
                f'Saved pose for tag_id={resolved_tag_id} is outside workspace limits: '
                f'x={x_mm:.1f}, y={y_mm:.1f}, z={z_mm:.1f} mm'
            )

        ret = self._set_gripper_position(self.gripper_close_pos)
        if ret != 0:
            return False, f'Failed to keep gripper closed before return move (code={ret}).'

        # Add a few intermediate waypoints for a smoother return trajectory if obstacles are in the way (based on observed behavior during testing)
        ret = self._move_to_mm(x_mm, y_mm + 100.0*math.sin(math.radians(-45)), z_mm + 100.0*math.cos(math.radians(-45)), roll, pitch, yaw)
        if ret != 0:
            return False, f'Failed to move to raised pose (code={ret}).'

        # ret = self._vibrate_arm()
        # if ret != 0:
        #     return False, f'Failed to vibrate arm to level food (code={ret}).'

        ret = self._move_to_mm(x_mm, y_mm, z_mm, roll, pitch, yaw)
        if ret != 0:
            return False, (
                f'Failed to move to saved initial pose for tag_id={resolved_tag_id} '
                f'(code={ret}).'
            )

        message = (
            f'Moved to saved initial pose for tag_id={resolved_tag_id} at {self.target_frame}: '
            f'x={x_mm:.1f}mm, y={y_mm:.1f}mm, z={z_mm:.1f}mm (gripper closed).'
        )

        idle_ok, idle_err = self._wait_for_arm_idle(timeout_sec=10.0)
        if not idle_ok:
            return False, f'Return motion completion check failed: {idle_err}'

        self.get_logger().info(message)
        return True, message

    def _execute_grasp(self):
        if not self._grasp_lock.acquire(blocking=False):
            return False, 'Grasp already in progress.'

        approach_6dof  = [429.1, 31.5, -77.7, math.radians(-168.7), math.radians(-44.4), math.radians(-88.7)]

        try:
            selected_idx, resolved_tag_id, map_error = self._resolve_selected_bowl_tag()
            if map_error is not None:
                return False, map_error

            # Save previous checked volume for this bowl before marking it as pending.
            prev_volume_m3 = self._volume_check_status_by_bowl.get(selected_idx, {}).get('last_detected_food_volume_m3', 0.0)
            if prev_volume_m3 == 0.0:
                self._expected_volume_m3[selected_idx] = self.init_expected_volume_m3
                self.get_logger().info(
                    f'No previous volume check for selected_bowl_idx={selected_idx}. '
                    f'Setting expected volume to {self._expected_volume_m3[selected_idx]*1e6:.3e} ml.'
                )
            # Mark only this bowl's result as pending while preserving the most
            # recent results for every other bowl.
            self._update_volume_check_status(selected_idx, 0.0, False)

            if resolved_tag_id < 0:
                message = (
                    f'Skipping arm2 grasp for selected_bowl_idx={selected_idx}: '
                    f'mapped tag_id={resolved_tag_id} (no-tag bowl).'
                )
                self.get_logger().info(message)
                return True, message

            self.get_logger().info(
                f'Resolved selected_bowl_idx={selected_idx} to AprilTag ID {resolved_tag_id}.'
            )

            apriltag_pose = None
            apriltag_quat = None
            apriltag_frame = None
            apriltag_error = None
            skip_regrasp = False

            for attempt in range(1, self.apriltag_request_retries + 2):
                apriltag_pose, apriltag_quat, apriltag_frame, apriltag_error = self._lookup_apriltag_target_tf(
                    resolved_tag_id
                )
                if apriltag_pose is not None:
                    break
                self.get_logger().warn(
                    f'AprilTag TF attempt {attempt}/{self.apriltag_request_retries + 1} '
                    f'failed for tag_id={resolved_tag_id}: {apriltag_error}'
                )

            if apriltag_pose is None:
                ret = self._move_to_mm(
                    approach_6dof[0], approach_6dof[1], approach_6dof[2],
                    approach_6dof[3], approach_6dof[4], approach_6dof[5],
                )
                if ret != 0:
                    return False, f'Failed to move to approach pose (code={ret}).'
                return False, f'AprilTag targeting failed: {apriltag_error}'

            source_name = f'apriltag_tf:{apriltag_frame}'
            x_m = apriltag_pose[0] + float(self.point_offset_xyz[0])
            y_m = apriltag_pose[1] + float(self.point_offset_xyz[1])
            z_m = (
                apriltag_pose[2]
                + float(self.point_offset_xyz[2])
                + self.grasp_z_offset
            )

            if self.use_apriltag_orientation:
                q = apriltag_quat
                roll, pitch, yaw = quat_to_rpy(
                    float(q[0]),
                    float(q[1]),
                    float(q[2]),
                    float(q[3]),
                )
            else:
                roll, pitch, yaw = self._get_orientation()

            # init_pose_6dof = [
            #     x_m * 1000.0,
            #     y_m * 1000.0,
            #     z_m * 1000.0,
            #     roll,
            #     pitch,
            #     yaw,
            # ]
            # self._saved_init_pose_by_tag[resolved_tag_id] = init_pose_6dof
            # self.get_logger().info(
            #     f'Saved initial pose for tag_id={resolved_tag_id}: '
            #     f'x={init_pose_6dof[0]:.1f}mm, y={init_pose_6dof[1]:.1f}mm, z={init_pose_6dof[2]:.1f}mm'
            # )

            if selected_idx == self._last_successful_selected_bowl_idx:
                skip_regrasp = True

            x_mm = x_m * 1000.0
            y_mm = y_m * 1000.0
            z_grasp_mm = z_m * 1000.0
            z_approach_mm = z_grasp_mm + (self.approach_z_offset * 1000.0)
            z_lift_mm = z_grasp_mm + (self.lift_z_offset * 1000.0)

            if skip_regrasp:
                message = (f'selected_bowl_idx={selected_idx} matches previous successful grasp. '
                    'Skipping re-grasp motions and moving directly to final tilt pose.')
                self.get_logger().info(message)
                ret, init_pose_6dof = self.arm.get_position(is_radian=True)
                if ret == 0 and init_pose_6dof is not None and len(init_pose_6dof) >= 6:
                    self._saved_init_pose_by_tag[resolved_tag_id] = init_pose_6dof
                    self.get_logger().info(
                        f'Saved initial pose for tag_id={resolved_tag_id}: '
                        f'x={init_pose_6dof[0]:.1f}mm, y={init_pose_6dof[1]:.1f}mm, z={init_pose_6dof[2]:.1f}mm'
                    )
                    x_mm, y_mm, z_grasp_mm, roll, pitch, yaw = init_pose_6dof
            else:
                ret = self._set_gripper_position(self.gripper_open_pos)
                if ret != 0:
                    return False, f'Failed to open gripper (code={ret}).'

                # Fixed approach pose away from target to reduce risk of collision during approach
                # x = 400.5mm, y = 18.2mm, z = -68.5mm, roll = -179.6 deg, pitch = -45.3 deg, yaw = -90.2 deg
                # ret = self._move_to_mm(x_mm, y_mm, z_approach_mm, roll, pitch, yaw)
                ret = self._move_to_mm(approach_6dof[0], approach_6dof[1], approach_6dof[2], approach_6dof[3], approach_6dof[4], approach_6dof[5])
                if ret != 0:
                    return False, f'Failed to move to approach pose (code={ret}).'

                if not self._within_workspace(x_mm, y_mm, z_grasp_mm):
                    return False, (
                        f'Target outside workspace limits: x={x_mm:.1f}, y={y_mm:.1f}, z={z_grasp_mm:.1f} mm'
                    )

                self.get_logger().info(
                    f'Executing grasp using {source_name} at {self.target_frame}: '
                    f'x={x_mm:.1f}mm, y={y_mm:.1f}mm, z={z_grasp_mm:.1f}mm, '
                    f'roll={math.degrees(roll):.1f}deg, pitch={math.degrees(pitch):.1f}deg, yaw={math.degrees(yaw):.1f}deg'
                )

                ret = self._move_to_mm(x_mm, y_mm, z_grasp_mm, roll, pitch, yaw)
                if ret != 0:
                    #Clear error and move back to approach pose to avoid collision with bowl
                    self.arm.clean_error()
                    self.arm.motion_enable(enable=True) 
                    self.arm.set_state(0)
                    ret = self._move_to_mm(approach_6dof[0], approach_6dof[1], approach_6dof[2], approach_6dof[3], approach_6dof[4], approach_6dof[5])
                    return False, f'Failed to move to grasp pose (code={ret}).'

                ret = self._set_gripper_position(self.gripper_close_pos)
                if ret != 0:
                    return False, f'Failed to close gripper (code={ret}).'

                message = (
                    f'Grasp succeeded using {source_name} at {self.target_frame}: '
                    f'x={x_mm:.1f}mm, y={y_mm:.1f}mm, z={z_grasp_mm:.1f}mm'
                )
                self.get_logger().info(message)

                ret, init_pose_6dof = self.arm.get_position(is_radian=True)
                if ret == 0 and init_pose_6dof is not None and len(init_pose_6dof) >= 6:
                    self._saved_init_pose_by_tag[resolved_tag_id] = init_pose_6dof
                    self.get_logger().info(
                        f'Saved initial pose for tag_id={resolved_tag_id}: '
                        f'x={init_pose_6dof[0]:.1f}mm, y={init_pose_6dof[1]:.1f}mm, z={init_pose_6dof[2]:.1f}mm'
                    )

            # TODO: RAISE BOWL UP FIRST
            # Raise the bowl vertically up first to avoid collisions during tilt
            # Coordinate frame is 45 deg rotated anticlockwise, so x is forward/backward, y is diagnal left/right, z is  diagonal up/down 
            # Move the arm up by 100mm vertically up first before tilting to reduce risk of collision with other bowls
            ret=self._move_to_mm(x_mm, y_mm + 100.0*math.sin(math.radians(-45)), z_grasp_mm + 100.0*math.cos(math.radians(-45)), roll, pitch, yaw)
            if ret != 0:
                return False, f'Failed to move to raised pose (code={ret}).'

            # Move to the predetermined pose from which to estimate the bowl food volume.
            vol_pose = self.volume_estimation_pose_6dof
            ret = self._move_to_mm(vol_pose[0], vol_pose[1], vol_pose[2], vol_pose[3], vol_pose[4], vol_pose[5])
            if ret != 0:
                return False, f'Failed to move to volume-estimation pose (code={ret}).'

            idle_ok, idle_err = self._wait_for_arm_idle(timeout_sec=10.0)
            if not idle_ok:
                return False, f'Volume-estimation pose completion check failed: {idle_err}'

            # ret = self._vibrate_arm()
            # if ret != 0:
            #     return False, f'Failed to vibrate arm to level food (code={ret}).'

            time.sleep(3.0)  # Allow time for the camera to settle and capture a clear point-cloud

            # This service is intentionally called only after the xArm has reached
            # and settled at volume_estimation_pose_6dof.
            volume_m3, volume_err = self._call_get_bowl_food_ratio()
            if volume_m3 is None:
                return False, f'Failed to estimate bowl food volume: {volume_err}'
            self.get_logger().info(
                f'Detected food volume for selected_bowl_idx={selected_idx} is {volume_m3:.3e} m^3')
            
            volume_m3 = volume_m3 + self.volume_offset_ml * 1e-6  # Convert offset from ml to m^3

            for i in range(3):
                if volume_m3 > self._expected_volume_m3[selected_idx] + 1e-5 or volume_m3 < self._expected_volume_m3[selected_idx] - 1e-5:
                    self.get_logger().warn(
                        f'Detected food volume (with offset) ({volume_m3:.3e} m^3) is not within expected range of +/- 1e-5m^3 of'
                        f'({self._expected_volume_m3[selected_idx]:.3e} m^3); re-checking volume after a brief wait.'
                    )
                    time.sleep(1.0)
                    volume_m3, volume_err = self._call_get_bowl_food_ratio()
                    if volume_m3 is None:
                        return False, f'Failed to estimate bowl food volume: {volume_err}'
                    volume_m3 = volume_m3 + self.volume_offset_ml * 1e-6
                else:
                    self.get_logger().info(
                        f'Detected food volume (with offset) ({volume_m3:.3e} m^3) is within expected range of +/- 1e-5m^3 of'
                        f'({self._expected_volume_m3[selected_idx]:.3e} m^3); proceeding with tilt computation.'
                    )
                    break
            
            if volume_m3 > self._expected_volume_m3[selected_idx] + 1e-5 or volume_m3 < self._expected_volume_m3[selected_idx] - 1e-5:
                self.get_logger().warn(
                    f'Detected food volume (with offset) ({volume_m3:.3e} m^3) is still not within expected range of +/- 1e-5m^3 of'
                    f'({self._expected_volume_m3[selected_idx]:.3e} m^3) after 3 checks; using the EXPECTED VOLUME for tilt computation.'
                )
                volume_m3 = self._expected_volume_m3[selected_idx]

            # for i in range(3):
            #     if prev_volume_m3 > 0.0:
            #         if volume_m3 > prev_volume_m3:
            #             self.get_logger().warn(
            #                 f'Detected food volume increased from {prev_volume_m3:.3e} m^3 to {volume_m3:.3e} m^3; '
            #                 're-checking volume after a brief wait.'
            #             )
            #             time.sleep(1.0)
            #             volume_m3, volume_err = self._call_get_bowl_food_ratio()
            #             if volume_m3 is None:
            #                 return False, f'Failed to estimate bowl food volume: {volume_err}'
            #         else:
            #             break
            #     else:
            #         break
            # if prev_volume_m3 > 0.0 and volume_m3 > prev_volume_m3:
            #     self.get_logger().warn(
            #         f'Detected food volume increased from {prev_volume_m3:.3e} m^3 to {volume_m3:.3e} m^3 after 3 checks; '
            #         'using the latest detected volume for tilt computation.'
            #     )
            #     volume_m3 = prev_volume_m3 - self.target_scoop_ml * 1e-6 


            self._update_volume_check_status(selected_idx, volume_m3, True)
            if volume_m3 < self.minimum_food_volume_m3:
                return False, (
                    f'Detected food volume ({volume_m3:.3e} m^3) is below the minimum '
                    f'({self.minimum_food_volume_m3:.3e} m^3); skipping tilt-bowl scooping.'
                )

            # food_mass_g = self._volume_mass_model.predict(volume_m3)
            # Convert volume_m3 to ml
            volume_ml = volume_m3 * 1e6 #+ self.volume_offset_ml

            best_angle_deg = self._tilt_optimiser[selected_idx].get_optimal_tilt(volume_ml, self.target_scoop_ml)
            if best_angle_deg is None:
                return False, (
                    f'Tilt optimiser failed to compute an angle '
                    f'(volume={volume_m3:.3e} m^3, {volume_ml:.1f} ml, target={self.target_scoop_ml:.2f} ml).'
                )
            best_angle_deg = float(best_angle_deg)

            self.get_logger().info(
                f'Adaptive tilt: detected_volume={volume_m3:.3e} m^3, {volume_ml:.2f} ml, '
                f'target_scoop={self.target_scoop_ml:.2f} ml -> best_angle={best_angle_deg:.2f} deg.'
            )

            # Tilt the bowl to the computed pose.
            bowl_tilted_pose = get_new_bowl_pose(best_angle_deg)
            ret = self._move_to_mm(
                bowl_tilted_pose[0], bowl_tilted_pose[1], bowl_tilted_pose[2],
                bowl_tilted_pose[3], bowl_tilted_pose[4], bowl_tilted_pose[5],
            )
            if ret != 0:
                return False, f'Failed to move to tilt pose (code={ret}).'

            # ret = self._vibrate_arm()
            # if ret != 0:
            #     return False, f'Failed to vibrate arm (code={ret}).'

            # Publish the computed angle so the task_planner can drive the lite6 trajectory.
            self._update_last_tilt_angle_param(best_angle_deg)

            # Do not report success until arm motion has fully settled.
            idle_ok, idle_err = self._wait_for_arm_idle(timeout_sec=10.0)
            if not idle_ok:
                return False, f'Grasp completion check failed: {idle_err}'

            self._last_successful_selected_bowl_idx = selected_idx

            self._expected_volume_m3[selected_idx] = self._expected_volume_m3[selected_idx] - self.target_scoop_ml * 1e-6  # Update expected volume for next grasp

            return True, message
        except Exception as exc:
            message = f'Unhandled grasp failure: {exc}'
            self.get_logger().error(message)
            self.get_logger().info('Attempting to move to safe approach pose after failure.')
            self.arm.clean_error()
            self.arm.motion_enable(enable=True)
            self.arm.set_state(0)
            ret = self._move_to_mm(approach_6dof[0], approach_6dof[1], approach_6dof[2], approach_6dof[3], approach_6dof[4], approach_6dof[5])
            return False, message
        finally:
            self._grasp_lock.release()

    def execute_grasp_callback(self, _request, response):
        success, message = self._execute_grasp()
        response.success = success
        response.message = message
        return response

    def move_to_saved_initial_pose_callback(self, _request, response):
        if not self._grasp_lock.acquire(blocking=False):
            response.success = False
            response.message = 'Another grasp or return motion is already in progress.'
            return response

        try:
            success, message = self._move_to_saved_initial_pose()
            response.success = success
            response.message = message
            return response
        except Exception as exc:
            message = f'Unhandled return-to-initial-pose failure: {exc}'
            self.get_logger().error(message)
            response.success = False
            response.message = message
            return response
        finally:
            self._grasp_lock.release()

    def disconnect_arm(self):
        if self.arm is None:
            return

        try:
            self.arm.set_mode(0)
            self.arm.set_state(0)
            self.arm.disconnect()
        except Exception as exc:
            self.get_logger().warn(f'Exception while disconnecting arm: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = Arm2ScoopingGrasp()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, shutting down arm2_scooping_grasp.')
    finally:
        executor.shutdown()
        node.disconnect_arm()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
