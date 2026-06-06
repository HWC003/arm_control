#!/usr/bin/env python3
import math
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_srvs.srv import Trigger
from xarm.wrapper import XArmAPI


class Arm2ExtrinsicCalibrationHelper(Node):
    """Collect camera/arm point pairs and estimate camera->arm2 extrinsics."""

    def __init__(self):
        super().__init__('arm2_extrinsic_calibration_helper')

        self.declare_parameter('robot_ip', '192.168.1.201')
        self.declare_parameter('camera_frame', 'orbbec_camera_color_optical_frame')
        self.declare_parameter('camera_point_topic', '/clicked_point')
        self.declare_parameter('arm2_base_frame', 'xarm2_base')

        self.declare_parameter('capture_service_name', 'arm2/calibration/capture_pair')
        self.declare_parameter('solve_service_name', 'arm2/calibration/solve')
        self.declare_parameter('reset_service_name', 'arm2/calibration/reset')

        self.declare_parameter('min_pairs', 4)
        self.declare_parameter('eef_offset_xyz_mm', [0.0, 0.0, 0.0])

        self.robot_ip = self.get_parameter('robot_ip').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.camera_point_topic = self.get_parameter('camera_point_topic').value
        self.arm2_base_frame = self.get_parameter('arm2_base_frame').value

        self.capture_service_name = self.get_parameter('capture_service_name').value
        self.solve_service_name = self.get_parameter('solve_service_name').value
        self.reset_service_name = self.get_parameter('reset_service_name').value

        self.min_pairs = int(self.get_parameter('min_pairs').value)
        self.eef_offset_xyz_mm = [float(v) for v in self.get_parameter('eef_offset_xyz_mm').value]

        if len(self.eef_offset_xyz_mm) != 3:
            raise ValueError('eef_offset_xyz_mm must contain exactly 3 values.')

        self._pair_lock = threading.Lock()
        self._latest_point_msg = None
        self._camera_points = []
        self._arm_points = []

        self.create_subscription(PointStamped, self.camera_point_topic, self._point_cb, 10)

        self.create_service(Trigger, self.capture_service_name, self._capture_cb)
        self.create_service(Trigger, self.solve_service_name, self._solve_cb)
        self.create_service(Trigger, self.reset_service_name, self._reset_cb)

        self.arm = None
        self._connect_arm()

        self.get_logger().info('Arm2 extrinsic calibration helper started.')
        self.get_logger().info(
            'Workflow: publish PointStamped in camera frame -> call capture service -> repeat -> call solve service.'
        )

    def _connect_arm(self):
        self.arm = XArmAPI(port=self.robot_ip, is_radian=True)

    def _point_cb(self, msg: PointStamped):
        self._latest_point_msg = msg

    def _read_arm_point_m(self):
        ret, pose = self.arm.get_position(is_radian=True)
        if ret != 0 or pose is None or len(pose) < 3:
            return None, f'Failed to read arm pose (code={ret}).'

        x_mm = float(pose[0]) + self.eef_offset_xyz_mm[0]
        y_mm = float(pose[1]) + self.eef_offset_xyz_mm[1]
        z_mm = float(pose[2]) + self.eef_offset_xyz_mm[2]

        return np.array([x_mm, y_mm, z_mm], dtype=np.float64) / 1000.0, None

    def _capture_cb(self, _request, response):
        if self._latest_point_msg is None:
            response.success = False
            response.message = 'No camera point received yet on camera_point_topic.'
            return response

        if self._latest_point_msg.header.frame_id != self.camera_frame:
            response.success = False
            response.message = (
                f'Latest point frame mismatch: got={self._latest_point_msg.header.frame_id}, '
                f'expected={self.camera_frame}'
            )
            return response

        arm_point, err = self._read_arm_point_m()
        if arm_point is None:
            response.success = False
            response.message = err
            return response

        cam_point = np.array(
            [
                float(self._latest_point_msg.point.x),
                float(self._latest_point_msg.point.y),
                float(self._latest_point_msg.point.z),
            ],
            dtype=np.float64,
        )

        with self._pair_lock:
            self._camera_points.append(cam_point)
            self._arm_points.append(arm_point)
            pair_count = len(self._camera_points)

        response.success = True
        response.message = (
            f'Captured pair #{pair_count}: '
            f'camera=({cam_point[0]:.4f}, {cam_point[1]:.4f}, {cam_point[2]:.4f}) m, '
            f'arm=({arm_point[0]:.4f}, {arm_point[1]:.4f}, {arm_point[2]:.4f}) m'
        )
        self.get_logger().info(response.message)
        return response

    def _reset_cb(self, _request, response):
        with self._pair_lock:
            self._camera_points = []
            self._arm_points = []
        response.success = True
        response.message = 'Calibration pairs cleared.'
        self.get_logger().info(response.message)
        return response

    def _solve_cb(self, _request, response):
        with self._pair_lock:
            cam = np.array(self._camera_points, dtype=np.float64)
            arm = np.array(self._arm_points, dtype=np.float64)

        pair_count = len(cam)
        if pair_count < self.min_pairs:
            response.success = False
            response.message = (
                f'Need at least {self.min_pairs} pairs, but only {pair_count} captured.'
            )
            return response

        try:
            rot, trans = self._estimate_rigid_transform(cam, arm)
            rmse = self._compute_rmse(cam, arm, rot, trans)
            roll, pitch, yaw = self._rot_to_rpy(rot)
            qx, qy, qz, qw = self._rot_to_quat(rot)

            self.get_logger().info('Estimated camera->arm2 transform:')
            self.get_logger().info(
                f'translation (m): x={trans[0]:.6f}, y={trans[1]:.6f}, z={trans[2]:.6f}'
            )
            self.get_logger().info(
                f'rotation rpy (rad): roll={roll:.6f}, pitch={pitch:.6f}, yaw={yaw:.6f}'
            )
            self.get_logger().info(
                f'rotation quat: qx={qx:.6f}, qy={qy:.6f}, qz={qz:.6f}, qw={qw:.6f}'
            )
            self.get_logger().info(f'fit rmse: {rmse:.6f} m from {pair_count} pairs')

            static_tf_cmd = (
                'ros2 run tf2_ros static_transform_publisher '
                f'--x {trans[0]:.6f} --y {trans[1]:.6f} --z {trans[2]:.6f} '
                f'--roll {roll:.6f} --pitch {pitch:.6f} --yaw {yaw:.6f} '
                f'--frame-id {self.camera_frame} --child-frame-id {self.arm2_base_frame}'
            )
            launch_cmd = (
                'ros2 launch arm_control_pkg arm2_scooping_grasp.launch.py '
                'publish_camera_to_arm2_tf:=true '
                f'camera_source_frame:={self.camera_frame} '
                f'arm2_base_frame:={self.arm2_base_frame} '
                f'camera_to_arm2_tx:={trans[0]:.6f} '
                f'camera_to_arm2_ty:={trans[1]:.6f} '
                f'camera_to_arm2_tz:={trans[2]:.6f} '
                f'camera_to_arm2_roll:={roll:.6f} '
                f'camera_to_arm2_pitch:={pitch:.6f} '
                f'camera_to_arm2_yaw:={yaw:.6f}'
            )

            self.get_logger().info('Static TF command:')
            self.get_logger().info(static_tf_cmd)
            self.get_logger().info('Launch command with estimated TF:')
            self.get_logger().info(launch_cmd)

            response.success = True
            response.message = (
                f'Solved with {pair_count} pairs; rmse={rmse:.6f} m; '
                f't=({trans[0]:.4f}, {trans[1]:.4f}, {trans[2]:.4f})'
            )
            return response
        except Exception as exc:
            response.success = False
            response.message = f'Failed to solve extrinsics: {exc}'
            return response

    @staticmethod
    def _estimate_rigid_transform(src_pts, dst_pts):
        src_center = np.mean(src_pts, axis=0)
        dst_center = np.mean(dst_pts, axis=0)

        src_zero = src_pts - src_center
        dst_zero = dst_pts - dst_center

        h_mat = src_zero.T @ dst_zero
        u_mat, _, vt_mat = np.linalg.svd(h_mat)

        rot = vt_mat.T @ u_mat.T
        if np.linalg.det(rot) < 0:
            vt_mat[2, :] *= -1
            rot = vt_mat.T @ u_mat.T

        trans = dst_center - (rot @ src_center)
        return rot, trans

    @staticmethod
    def _compute_rmse(src_pts, dst_pts, rot, trans):
        pred = (rot @ src_pts.T).T + trans
        err = pred - dst_pts
        return math.sqrt(float(np.mean(np.sum(err * err, axis=1))))

    @staticmethod
    def _rot_to_rpy(rot):
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

    @staticmethod
    def _rot_to_quat(rot):
        trace = rot[0, 0] + rot[1, 1] + rot[2, 2]

        if trace > 0.0:
            s_val = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s_val
            qx = (rot[2, 1] - rot[1, 2]) / s_val
            qy = (rot[0, 2] - rot[2, 0]) / s_val
            qz = (rot[1, 0] - rot[0, 1]) / s_val
        elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
            s_val = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
            qw = (rot[2, 1] - rot[1, 2]) / s_val
            qx = 0.25 * s_val
            qy = (rot[0, 1] + rot[1, 0]) / s_val
            qz = (rot[0, 2] + rot[2, 0]) / s_val
        elif rot[1, 1] > rot[2, 2]:
            s_val = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
            qw = (rot[0, 2] - rot[2, 0]) / s_val
            qx = (rot[0, 1] + rot[1, 0]) / s_val
            qy = 0.25 * s_val
            qz = (rot[1, 2] + rot[2, 1]) / s_val
        else:
            s_val = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
            qw = (rot[1, 0] - rot[0, 1]) / s_val
            qx = (rot[0, 2] + rot[2, 0]) / s_val
            qy = (rot[1, 2] + rot[2, 1]) / s_val
            qz = 0.25 * s_val

        return qx, qy, qz, qw

    def shutdown(self):
        if self.arm is None:
            return
        try:
            self.arm.disconnect()
        except Exception as exc:
            self.get_logger().warn(f'Error during arm disconnect: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = Arm2ExtrinsicCalibrationHelper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down arm2 extrinsic calibration helper.')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
