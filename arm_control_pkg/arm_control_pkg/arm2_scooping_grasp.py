#!/usr/bin/env python3
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_geometry_msgs import do_transform_point
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)

from feeding_msgs.srv import GetScoopingPoint
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
    1) Request AprilTag target pose (optional primary source).
    2) If unavailable, optionally fall back to GetScoopingPoint bowl centroid.
    3) Convert target pose to xArm command pose.
    4) Execute open -> approach -> close -> lift sequence.
    5) If selected bowl maps to no tag, skip arm motion as a no-op.
    """

    def __init__(self):
        super().__init__('arm2_scooping_grasp')

        # Core connectivity
        self.declare_parameter('robot_ip', '192.168.1.201')
        self.declare_parameter('get_scooping_point_service', 'food_perception/get_scooping_point')
        self.declare_parameter('trigger_service_name', 'arm2/execute_grasp')
        self.declare_parameter('return_to_init_service_name', 'arm2/return_to_saved_initial_pose')
        self.declare_parameter('selected_bowl_idx', -1)
        self.declare_parameter('auto_execute_on_index_set', True)
        self.declare_parameter('target_frame', 'xarm2_base')
        self.declare_parameter('tf_timeout_sec', 1.0)
        self.declare_parameter('service_timeout_sec', 5.0)
        self.declare_parameter('request_retries', 2)
        self.declare_parameter('use_apriltag_target', True)
        self.declare_parameter('fallback_to_bowl_centroid', False)
        self.declare_parameter('apriltag_target_id', 0)
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

        # Optional workspace safety limits in mm (target frame)
        self.declare_parameter('use_workspace_limits', False)
        self.declare_parameter('workspace_min_xyz', [200.0, -400.0, 80.0])
        self.declare_parameter('workspace_max_xyz', [700.0, 400.0, 500.0])

        self.robot_ip = self.get_parameter('robot_ip').value
        self.scooping_service_name = self.get_parameter('get_scooping_point_service').value
        self.trigger_service_name = self.get_parameter('trigger_service_name').value
        self.return_to_init_service_name = self.get_parameter('return_to_init_service_name').value
        self.target_frame = self.get_parameter('target_frame').value
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)
        self.service_timeout_sec = float(self.get_parameter('service_timeout_sec').value)
        self.request_retries = int(self.get_parameter('request_retries').value)
        self.use_apriltag_target = bool(self.get_parameter('use_apriltag_target').value)
        self.fallback_to_bowl_centroid = bool(self.get_parameter('fallback_to_bowl_centroid').value)
        self.apriltag_target_id = int(self.get_parameter('apriltag_target_id').value)
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

        self.gripper_open_pos = float(self.get_parameter('gripper_open_pos').value)
        self.gripper_close_pos = float(self.get_parameter('gripper_close_pos').value)
        self.gripper_speed = float(self.get_parameter('gripper_speed').value)
        self.auto_execute_on_index_set = bool(self.get_parameter('auto_execute_on_index_set').value)

        self.use_workspace_limits = bool(self.get_parameter('use_workspace_limits').value)
        self.workspace_min_xyz = list(self.get_parameter('workspace_min_xyz').value)
        self.workspace_max_xyz = list(self.get_parameter('workspace_max_xyz').value)

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

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._scooping_client = self.create_client(
            GetScoopingPoint,
            self.scooping_service_name,
            callback_group=self._callback_group,
        )

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

        return result

    def _auto_execute_timer_cb(self):
        if not self.auto_execute_on_index_set:
            return
        if not self._pending_auto_execute:
            return

        if self._grasp_lock.locked():
            return

        idx = self._pending_auto_idx
        self._pending_auto_execute = False

        success, message = self._execute_grasp()
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

    def _call_get_scooping_point(self):
        if not self._scooping_client.wait_for_service(timeout_sec=1.0):
            return None, 'GetScoopingPoint service is unavailable.'

        req = GetScoopingPoint.Request()
        future = self._scooping_client.call_async(req)

        deadline = time.time() + self.service_timeout_sec
        while rclpy.ok() and not future.done() and time.time() < deadline:
            time.sleep(0.02)

        if not future.done():
            return None, f'GetScoopingPoint call timed out after {self.service_timeout_sec:.1f}s.'

        if future.exception() is not None:
            return None, f'GetScoopingPoint call failed: {future.exception()}'

        resp = future.result()
        if resp is None:
            return None, 'GetScoopingPoint returned no response.'

        if not resp.success:
            return None, 'GetScoopingPoint returned success=false.'

        if len(resp.bowl_centroids) == 0:
            return None, 'GetScoopingPoint returned no bowl centroids.'

        idx = int(self.get_parameter('selected_bowl_idx').value)
        if idx < 0:
            return None, (
                f'selected_bowl_idx is {idx}. Manager must set selected_bowl_idx before calling trigger.'
            )

        if idx < 0 or idx >= len(resp.bowl_centroids):
            return None, (
                f'selected_bowl_idx out of bounds: {idx} for {len(resp.bowl_centroids)} centroids.'
            )

        self.get_logger().info(
            f'Using bowl centroid at selected_bowl_idx={idx} as grasp target source.'
        )

        return resp.bowl_centroids[idx], None

    def _transform_point_to_target(self, src_point: PointStamped):
        source_frame = src_point.header.frame_id
        if not source_frame:
            return None, 'PointStamped has empty frame_id.'

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
            transformed = do_transform_point(src_point, transform)
            return transformed, None
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            return None, (
                f'TF lookup failed for {source_frame} -> {self.target_frame}: {exc}'
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
        amplitude = 3.0  # mm
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

        ret = self._vibrate_arm()
        if ret != 0:
            return False, f'Failed to vibrate arm to level food (code={ret}).'

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
        self.get_logger().info(message)
        return True, message

    def _execute_grasp(self):
        if not self._grasp_lock.acquire(blocking=False):
            return False, 'Grasp already in progress.'

        try:
            selected_idx, resolved_tag_id, map_error = self._resolve_selected_bowl_tag()
            if map_error is not None:
                return False, map_error

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

            source_name = 'bowl_centroid'
            x_m = None
            y_m = None
            z_m = None
            roll = None
            pitch = None
            yaw = None

            apriltag_pose = None
            apriltag_quat = None
            apriltag_frame = None
            apriltag_error = None



            if self.use_apriltag_target:
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

                if apriltag_pose is not None:
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
                elif not self.fallback_to_bowl_centroid:
                    return False, f'AprilTag targeting failed and fallback disabled: {apriltag_error}'

            if x_m is None:
                point_msg = None
                last_error = None
                for attempt in range(1, self.request_retries + 2):
                    point_msg, last_error = self._call_get_scooping_point()
                    if point_msg is not None:
                        break
                    self.get_logger().warn(
                        f'GetScoopingPoint attempt {attempt}/{self.request_retries + 1} failed: {last_error}'
                    )

                if point_msg is None:
                    if apriltag_error is not None:
                        return False, (
                            f'Both target sources failed. AprilTag error: {apriltag_error}; '
                            f'Scooping error: {last_error}'
                        )
                    return False, f'Failed to get scooping point: {last_error}'

                transformed_point, tf_error = self._transform_point_to_target(point_msg)
                if transformed_point is None:
                    return False, tf_error

                x_m = transformed_point.point.x + float(self.point_offset_xyz[0])
                y_m = transformed_point.point.y + float(self.point_offset_xyz[1])
                z_m = transformed_point.point.z + float(self.point_offset_xyz[2]) + self.grasp_z_offset
                roll, pitch, yaw = self._get_orientation()

            init_pose_6dof = [
                x_m * 1000.0,
                y_m * 1000.0,
                z_m * 1000.0,
                roll,
                pitch,
                yaw,
            ]
            self._saved_init_pose_by_tag[resolved_tag_id] = init_pose_6dof
            self.get_logger().info(
                f'Saved initial pose for tag_id={resolved_tag_id}: '
                f'x={init_pose_6dof[0]:.1f}mm, y={init_pose_6dof[1]:.1f}mm, z={init_pose_6dof[2]:.1f}mm'
            )

            ret = self._set_gripper_position(self.gripper_open_pos)
            if ret != 0:
                return False, f'Failed to open gripper (code={ret}).'

            # Fixed approach pose away from target to reduce risk of collision during approach
            # x = 400.5mm, y = 18.2mm, z = -68.5mm, roll = -179.6 deg, pitch = -45.3 deg, yaw = -90.2 deg
            # ret = self._move_to_mm(x_mm, y_mm, z_approach_mm, roll, pitch, yaw)
            approach_6dof  = [470.8, -12.9, -120.8, math.radians(-176.7), math.radians(-42.3), math.radians(-90.6)]
            ret = self._move_to_mm(approach_6dof[0], approach_6dof[1], approach_6dof[2], approach_6dof[3], approach_6dof[4], approach_6dof[5])
            if ret != 0:
                return False, f'Failed to move to approach pose (code={ret}).'



            x_mm = x_m * 1000.0
            y_mm = y_m * 1000.0
            z_grasp_mm = z_m * 1000.0
            z_approach_mm = z_grasp_mm + (self.approach_z_offset * 1000.0)
            z_lift_mm = z_grasp_mm + (self.lift_z_offset * 1000.0)

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
                return False, f'Failed to move to grasp pose (code={ret}).'

            ret = self._set_gripper_position(self.gripper_close_pos)
            if ret != 0:
                return False, f'Failed to close gripper (code={ret}).'

            message = (
                f'Grasp succeeded using {source_name} at {self.target_frame}: '
                f'x={x_mm:.1f}mm, y={y_mm:.1f}mm, z={z_grasp_mm:.1f}mm'
            )
            self.get_logger().info(message)

            # TODO: RAISE BOWL UP FIRST
            # Raise the bowl vertically up first to avoid collisions during tilt
            # Coordinate frame is 45 deg rotated anticlockwise, so x is forward/backward, y is diagnal left/right, z is  diagonal up/down 
            # Move the arm up by 100mm vertically up first before tilting to reduce risk of collision with other bowls
            ret=self._move_to_mm(x_mm, y_mm + 100.0*math.sin(math.radians(-45)), z_grasp_mm + 100.0*math.cos(math.radians(-45)), roll, pitch, yaw)
            if ret != 0:
                return False, f'Failed to move to raised pose (code={ret}).'

            # Final tilt bowl pose 
            final_tilt_6dof = [386.7, -379.2, -101.6, math.radians(166.9), math.radians(-15.2), math.radians(-62.8)]
            ret = self._move_to_mm(final_tilt_6dof[0], final_tilt_6dof[1], final_tilt_6dof[2], final_tilt_6dof[3], final_tilt_6dof[4], final_tilt_6dof[5])
            if ret != 0:
                return False, f'Failed to move to tilt pose (code={ret}).'
            
            ret = self._vibrate_arm()
            if ret != 0:
                return False, f'Failed to vibrate arm (code={ret}).'

            return True, message
        except Exception as exc:
            message = f'Unhandled grasp failure: {exc}'
            self.get_logger().error(message)
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
