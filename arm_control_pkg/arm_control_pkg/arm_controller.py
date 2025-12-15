#!/usr/bin/env python

import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Bool
from scipy.spatial.transform import Rotation as R
from xarm.wrapper import XArmAPI

"""
Handles the physical xArm6 robot arm 
during the feeding sequence
"""

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller_server')
        self._shutdown_requested = False
        
        pkg_path = get_package_share_directory('arm_controller')

        self.get_logger().info("Starting Arm Controller Action Server")

        # Robot Parameters
        self.declare_parameter('/robot_type', 'xarm6')
        self.declare_parameter('/robot_ip', '192.168.1.201')

        self.robot_type = self.get_parameter('/robot_type').get_parameter_value().string_value
        self.robot_ip = self.get_parameter('/robot_ip').get_parameter_value().string_value

        self.declare_parameter('/default_speed', 150)
        self.declare_parameter('/default_accel', 80)

        self.default_speed = self.get_parameter('/default_speed').get_parameter_value().integer_value
        self.default_accel = self.get_parameter('/default_accel').get_parameter_value().integer_value

        # Robot Poses
        self.declare_parameter('poses.bite_transfer_start_pose', [471.5, 0.0, 388.0, 0.0, -1.571, 3.141])
        self.declare_parameter('poses.perception_pose', [400.0, 200.0, 325.0, -2.222, -0.245, -1.906])
        self.declare_parameter('poses.reset_pose', [471.5, 0.0, 388.0, 0.0, -1.571, 3.141])

        self._transfer_pose = self.get_parameter('poses.bite_transfer_start_pose').get_parameter_value().double_array_value
        self._perception_pose = self.get_parameter('poses.perception_pose').get_parameter_value().double_array_value
        self._reset_pose = self.get_parameter('poses.reset_pose').get_parameter_value().double_array_value

        # Subscribers
        self.sub_desired_pose = self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.pose_callback,
            10
        )

        self.sub_pause_cmd = self.create_subscription(
            Bool,
            'pause_cmd',
            self.pause_callback,
            1
        )

        self.is_paused = False
        self.setup_arm()

    def setup_arm(self, reset=False):
        """
        Set up the xArm6 Robot arm
        """
        self.arm = XArmAPI(port=self.robot_ip, is_radian=True)
        time.sleep(0.1)
        ready = self.arm.motion_enable(enable=True)
        if ready != 0:
            max_retries = 5
            for i in range (1, max_retries+1):
                ready = self.arm.motion_enable(enable=True)
                print(f"Trying to enable motion {i}/{max_retries}: {ready}")
                if ready:
                    break
                time.sleep(0.1)
        self.arm.set_mode(0)
        self.arm.set_state(0)
        time.sleep(0.1)

        if reset:
            self.reset()

    def disconnect_arm(self, reset=False):
            """
            Disconnect the xArm robot.
            """        
            if reset:
                self.arm.reset(wait=True)
            
            self.arm.disconnect()
            print(">> Disconnected arm...")
            time.sleep(0.1)    

    def pose_callback(self, pose):
        """
        Move the robot to a given pose
        pose: geometry_msgs/PoseStamped
        """
        if self.is_paused:
            self.get_logger().warn("Ignoring new pose command because arm is currently paused")
        x_position = pose.pose.position.x
        y_position = pose.pose.position.y
        z_position = pose.pose.position.z
        quat = [pose.pose.orientation.x, 
                pose.pose.orientation.y, 
                pose.pose.orientation.z, 
                pose.pose.orientation.w]
        rotation = R.from_quat(quat)
        euler = rotation.as_euler('xyz', degrees=True)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        self.arm.set_position(x=x_position, y=y_position, z=z_position, 
                              roll=roll, pitch=pitch, yaw=yaw, 
                              speed=self.default_speed, 
                              mvacc=self.default_accel, 
                              radius=0, 
                              wait=False)
        
    def pause_callback(self, msg: Bool):
        """
        Callback to handle stop/paused requests
        """
        if msg.data is True and not self.is_paused:
            self.get_logger().warn("PAUSE COMMAND RECEIVED: Stopping the arm immediately.")
            self.arm.set_state(3)
            self.is_paused=True

        elif msg.data is False and self.is_paused:
            self.get_logger().info("RESUME COMMAND RECEIVED: Enabling Motion.")
            self.arm.set_state(0)

        elif msg.data is True and self.is_paused:
            self.get_logger().info("ALREADY PAUSED: Doing Nothing")
        
        else:
            pass

def main(args=None):
    rclpy.init(args=args)    
    ac = ArmController()

    executor = MultiThreadedExecutor()
    executor.add_node(ac)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    ac.get_logger().info("Closing arm_controller")
    ac.disconnect_arm()

    executor.shutdown()
    ac.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
