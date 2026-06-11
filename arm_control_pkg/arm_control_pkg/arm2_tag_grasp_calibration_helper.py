#!/usr/bin/env python3
import json
import math
import threading

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R
from std_srvs.srv import Trigger
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)
from xarm.wrapper import XArmAPI


class Arm2TagGraspCalibrationHelper(Node):
    """Capture camera-tag and xArm6 pose pairs and solve camera->xArm transform."""

    def __init__(self):
        super().__init__('arm2_tag_grasp_calibration_helper')

        self.declare_parameter('robot_ip', '192.168.1.201')
        self.declare_parameter('camera_frame', 'orbbec_camera_color_optical_frame')
        self.declare_parameter('apriltag_target_id', 0)
        self.declare_parameter('tag_frame_templates', ['tag36h11:{id}', 'tag{id}', 'tag_{id}'])
        self.declare_parameter('tf_timeout_sec', 0.6)
        self.declare_parameter('max_tag_age_sec', 0.8)
        self.declare_parameter('strict_tag_age_check', True)

        self.declare_parameter('capture_service_name', 'arm2/tag_grasp_calibration/capture_pair')
        self.declare_parameter('solve_service_name', 'arm2/tag_grasp_calibration/solve')
        self.declare_parameter('reset_service_name', 'arm2/tag_grasp_calibration/reset')
        self.declare_parameter('dump_pairs_service_name', 'arm2/tag_grasp_calibration/dump_pairs')
        self.declare_parameter('min_samples', 4)

        self.declare_parameter('close_gripper_on_capture', True)
        self.declare_parameter('xarm_gripper_close_pos', 120.0)
        self.declare_parameter('xarm_gripper_speed', 2000.0)

        self.robot_ip = self.get_parameter('robot_ip').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.apriltag_target_id = int(self.get_parameter('apriltag_target_id').value)
        self.tag_frame_templates = [str(v) for v in self.get_parameter('tag_frame_templates').value]
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)
        self.max_tag_age_sec = float(self.get_parameter('max_tag_age_sec').value)
        self.strict_tag_age_check = bool(self.get_parameter('strict_tag_age_check').value)

        self.capture_service_name = self.get_parameter('capture_service_name').value
        self.solve_service_name = self.get_parameter('solve_service_name').value
        self.reset_service_name = self.get_parameter('reset_service_name').value
        self.dump_pairs_service_name = self.get_parameter('dump_pairs_service_name').value
        self.min_samples = int(self.get_parameter('min_samples').value)

        self.close_gripper_on_capture = bool(self.get_parameter('close_gripper_on_capture').value)
        self.xarm_gripper_close_pos = float(self.get_parameter('xarm_gripper_close_pos').value)
        self.xarm_gripper_speed = float(self.get_parameter('xarm_gripper_speed').value)

        if not self.tag_frame_templates:
            raise ValueError('tag_frame_templates must contain at least one template.')

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.arm = None
        self._connect_xarm()

        self._lock = threading.Lock()
        self._camera_points_mm = []
        self._xarm_points_mm = []
        self._tag_frames = []
        self._tag_ages_sec = []

        self.create_service(Trigger, self.capture_service_name, self._capture_cb)
        self.create_service(Trigger, self.solve_service_name, self._solve_cb)
        self.create_service(Trigger, self.reset_service_name, self._reset_cb)
        self.create_service(Trigger, self.dump_pairs_service_name, self._dump_pairs_cb)

        self.get_logger().info('arm2_tag_grasp_calibration_helper started.')
        self.get_logger().info(
            'Workflow: move tag + manually move xArm grasp pose -> capture_pair (repeat >=4) -> solve.'
        )

    def _connect_xarm(self):
        self.arm = XArmAPI(port=self.robot_ip, is_radian=True)
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(0)
        self.arm.set_gripper_enable(True)
        self.arm.set_gripper_mode(0)
        self.arm.set_gripper_speed(self.xarm_gripper_speed)

    @staticmethod
    def _time_to_sec(stamp) -> float:
        return float(stamp.sec) + (float(stamp.nanosec) * 1e-9)

    def _resolve_tag_frames(self):
        candidates = []
        for template in self.tag_frame_templates:
            try:
                frame = str(template).format(id=self.apriltag_target_id)
            except Exception:
                continue
            frame = frame.strip()
            if frame and frame not in candidates:
                candidates.append(frame)
        return candidates

    def _lookup_camera_to_tag_mm(self):
        candidates = self._resolve_tag_frames()
        if not candidates:
            return None, None, None, 'No valid tag frame candidates.'

        last_error = None
        for tag_frame in candidates:
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.camera_frame,
                    tag_frame,
                    Time(),
                    timeout=Duration(seconds=self.tf_timeout_sec),
                )

                now_sec = self.get_clock().now().nanoseconds * 1e-9
                stamp_sec = self._time_to_sec(tf_msg.header.stamp)
                age_sec = max(0.0, now_sec - stamp_sec) if stamp_sec > 0.0 else (self.max_tag_age_sec + 1.0)

                if self.strict_tag_age_check and age_sec > self.max_tag_age_sec:
                    return None, None, None, (
                        f'Tag pose stale for {tag_frame}: age={age_sec:.3f}s '
                        f'(limit={self.max_tag_age_sec:.3f}s).'
                    )

                tr = tf_msg.transform.translation
                cam_point_mm = np.array(
                    [float(tr.x), float(tr.y), float(tr.z)],
                    dtype=np.float64,
                ) * 1000.0

                return cam_point_mm, tag_frame, age_sec, None
            except (LookupException, ConnectivityException, ExtrapolationException) as exc:
                last_error = exc

        return None, None, None, f'Failed TF lookup for candidates {candidates}: {last_error}'

    def _read_xarm_xyz_mm(self):
        ret, pose = self.arm.get_position(is_radian=True)
        if ret != 0 or pose is None or len(pose) < 6:
            return None, f'Failed to read xArm pose (code={ret})'

        return np.array([float(pose[0]), float(pose[1]), float(pose[2])], dtype=np.float64), None

    @staticmethod
    def _is_non_colinear(points_mm: np.ndarray) -> bool:
        if points_mm.shape[0] < 3:
            return False
        centered = points_mm - np.mean(points_mm, axis=0)
        rank = np.linalg.matrix_rank(centered)
        return rank >= 2

    def _capture_cb(self, _request, response):
        with self._lock:
            cam_point_mm, tag_frame, age_sec, tf_err = self._lookup_camera_to_tag_mm()
            if cam_point_mm is None:
                response.success = False
                response.message = f'Capture failed (tag TF): {tf_err}'
                return response

            xarm_point_mm, arm_err = self._read_xarm_xyz_mm()
            if xarm_point_mm is None:
                response.success = False
                response.message = f'Capture failed (xArm pose): {arm_err}'
                return response

            if self.close_gripper_on_capture:
                self.arm.set_gripper_position(
                    self.xarm_gripper_close_pos,
                    wait=True,
                    speed=self.xarm_gripper_speed,
                    auto_enable=True,
                )

            self._camera_points_mm.append(cam_point_mm)
            self._xarm_points_mm.append(xarm_point_mm)
            self._tag_frames.append(tag_frame)
            self._tag_ages_sec.append(float(age_sec))

            n = len(self._camera_points_mm)
            response.success = True
            response.message = (
                f'Captured pair #{n}: camera_mm=({cam_point_mm[0]:.1f}, {cam_point_mm[1]:.1f}, {cam_point_mm[2]:.1f}), '
                f'xarm_mm=({xarm_point_mm[0]:.1f}, {xarm_point_mm[1]:.1f}, {xarm_point_mm[2]:.1f}), '
                f'tag_frame={tag_frame}, age={age_sec:.3f}s'
            )
            self.get_logger().info(response.message)
            return response

    def _solve_cb(self, _request, response):
        with self._lock:
            n = len(self._camera_points_mm)
            if n < self.min_samples:
                response.success = False
                response.message = f'Need at least {self.min_samples} samples, have {n}.'
                return response

            C = np.array(self._camera_points_mm, dtype=np.float64)
            B = np.array(self._xarm_points_mm, dtype=np.float64)

            if not self._is_non_colinear(C):
                response.success = False
                response.message = 'Camera points are colinear; collect more diverse samples.'
                return response

            if not self._is_non_colinear(B):
                response.success = False
                response.message = 'xArm points are colinear; collect more diverse samples.'
                return response

            centroid_C = np.mean(C, axis=0)
            centroid_B = np.mean(B, axis=0)

            C_centered = C - centroid_C
            B_centered = B - centroid_B

            H = C_centered.T @ B_centered
            U, S, Vt = np.linalg.svd(H)

            R_mat = Vt.T @ U.T
            if np.linalg.det(R_mat) < 0:
                Vt[2, :] *= -1
                R_mat = Vt.T @ U.T

            t = centroid_B - R_mat @ centroid_C

            T_matrix = np.eye(4)
            T_matrix[:3, :3] = R_mat
            T_matrix[:3, 3] = t

            quat = R.from_matrix(R_mat).as_quat()
            euler = R.from_matrix(R_mat).as_euler('xyz', degrees=True)

            pred = (R_mat @ C.T).T + t
            err = pred - B
            rmse_mm = math.sqrt(float(np.mean(np.sum(err * err, axis=1))))

            self.get_logger().info('--- CALIBRATION RESULTS ---')
            self.get_logger().info('\nOptimal 4x4 Transform Matrix (Camera to Base):')
            self.get_logger().info(f'\n{np.round(T_matrix, 4)}')
            self.get_logger().info('\nTranslation (x, y, z) in mm:')
            self.get_logger().info(f'{np.round(t, 2)}')
            self.get_logger().info('\nTranslation (x, y, z) in meters:')
            self.get_logger().info(f'{np.round(t / 1000.0, 3)}')
            self.get_logger().info('\nRotation in Quaternion (x, y, z, w):')
            self.get_logger().info(f'{np.round(quat, 5)}')
            self.get_logger().info('\nRotation in Euler Angles (RPY degrees):')
            self.get_logger().info(f'{np.round(euler, 2)}')
            self.get_logger().info('\nRotation in Euler Angles (RPY radians):')
            self.get_logger().info(f'{np.round(np.deg2rad(euler), 4)}')
            self.get_logger().info(f'\nFit RMSE (mm): {rmse_mm:.3f}')

            static_tf_cmd = (
                'ros2 run tf2_ros static_transform_publisher '
                f'--x {t[0] / 1000.0:.6f} --y {t[1] / 1000.0:.6f} --z {t[2] / 1000.0:.6f} '
                f'--roll {np.deg2rad(euler[0]):.6f} --pitch {np.deg2rad(euler[1]):.6f} --yaw {np.deg2rad(euler[2]):.6f} '
                f'--frame-id {self.camera_frame} --child-frame-id xarm2_base'
            )
            self.get_logger().info('Suggested static TF command:')
            self.get_logger().info(static_tf_cmd)

            result = {
                'num_samples': n,
                'camera_frame': self.camera_frame,
                'tag_frames': sorted(list(set(self._tag_frames))),
                'transform_matrix_camera_to_base': np.round(T_matrix, 6).tolist(),
                'translation_mm': np.round(t, 6).tolist(),
                'translation_m': np.round(t / 1000.0, 9).tolist(),
                'quaternion_xyzw': np.round(quat, 9).tolist(),
                'rpy_deg': np.round(euler, 9).tolist(),
                'rpy_rad': np.round(np.deg2rad(euler), 9).tolist(),
                'rmse_mm': float(rmse_mm),
            }

            response.success = True
            response.message = json.dumps(result)
            return response

    def _reset_cb(self, _request, response):
        with self._lock:
            self._camera_points_mm = []
            self._xarm_points_mm = []
            self._tag_frames = []
            self._tag_ages_sec = []

        response.success = True
        response.message = 'Calibration sample buffer reset.'
        self.get_logger().info(response.message)
        return response

    def _dump_pairs_cb(self, _request, response):
        with self._lock:
            pairs = []
            for i, (c_pt, b_pt, frame, age) in enumerate(
                zip(self._camera_points_mm, self._xarm_points_mm, self._tag_frames, self._tag_ages_sec),
                start=1,
            ):
                pairs.append(
                    {
                        'idx': i,
                        'camera_mm': [float(c_pt[0]), float(c_pt[1]), float(c_pt[2])],
                        'xarm_mm': [float(b_pt[0]), float(b_pt[1]), float(b_pt[2])],
                        'tag_frame': frame,
                        'tag_age_sec': float(age),
                    }
                )

            response.success = True
            response.message = json.dumps({'num_samples': len(pairs), 'pairs': pairs})
            return response

    def shutdown(self):
        if self.arm is None:
            return
        try:
            self.arm.disconnect()
        except Exception as exc:
            self.get_logger().warn(f'Error during xArm disconnect: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = Arm2TagGraspCalibrationHelper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down arm2_tag_grasp_calibration_helper.')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()