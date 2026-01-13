#!/usr/bin/env python
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool, String
from feeding_msgs.srv import CheckButtonState, SetArmMode, ImpedanceControl
from feeding_msgs.msg import SetPosition, GetPosition
from feeding_msgs.action import MoveToPose

from bite_transfer.ft_sensor_spoon import M8128TCPClient
from bite_transfer.sri_sensor_recorder import SensorRecorder

from xarm.wrapper import XArmAPI

import warnings

"""
Handles the physical xArm6 robot arm during the feeding sequence.

Organized into sections:
1. Initialization & ROS Setup
2. General xArm Functions
3, FT Sensor Functions
"""

class ArmController(Node):
    # ============================================================================
    # INITIALIZATION & ROS SETUP
    # ============================================================================
    
    def __init__(self):
        super().__init__('arm_controller_server')
        self._shutdown_requested = False
        
        pkg_path = get_package_share_directory('arm_control_pkg')

        self.get_logger().info("Starting Arm Controller Action Server")

        # Robot Parameters
        self.declare_parameter('/robot_type', 'xarm6')
        self.declare_parameter('/robot_ip', '192.168.1.201')

        self.robot_type = self.get_parameter('/robot_type').get_parameter_value().string_value
        self.robot_ip = self.get_parameter('/robot_ip').get_parameter_value().string_value

        self.declare_parameter('/default_speed', 100)
        self.declare_parameter('/default_accel', 50)

        self.default_speed = self.get_parameter('/default_speed').get_parameter_value().integer_value
        self.default_accel = self.get_parameter('/default_accel').get_parameter_value().integer_value

        # Robot Poses
        self.declare_parameter('poses.transfer_pose', [471.5, 0.0, 388.0, 0.0, -1.571, 3.141])
        self.declare_parameter('poses.perception_pose', [400.0, 200.0, 325.0, -2.222, -0.245, -1.906])
        self.declare_parameter('poses.reset_pose', [471.5, 0.0, 388.0, 0.0, -1.571, 3.141])

        self._transfer_pose = self.get_parameter('poses.transfer_pose').get_parameter_value().double_array_value
        self._perception_pose = self.get_parameter('poses.perception_pose').get_parameter_value().double_array_value
        self._reset_pose = self.get_parameter('poses.reset_pose').get_parameter_value().double_array_value

        # Bite Transfer Parameters
        self.declare_parameter("bite_threshold", 3.0)
        self._bite_threshold = self.get_parameter("bite_threshold").value  # Default threshold for bite detection
        self.declare_parameter("spoon_length", 0.195)
        self._spoon_length = self.get_parameter("spoon_length").value  # Spoon length in meters

        # Step counter for SetPosition
        self.setPosCount = 0

        # Callback Group
        self.parallel_group = ReentrantCallbackGroup()

        # Publishers
        self._current_pose_pub = self.create_publisher(GetPosition, '/current_pose', 10)

        # Subscribers
        self.move_to_point_sub = self.create_subscription(SetPosition, '/move_to_point', self.set_pose_callback, 10) #Used by bite_transfer.py and real_scooping_env.py to move to specific pose and position
        self.pause_cmd_sub = self.create_subscription(Bool, '/pause_cmd', self.pause_callback, 1)
        self.led_state_sub = self.create_subscription(Bool, '/led_state', self.set_led_callback, 1)

        #TODO: Remove this topic subscriber and use only the action server
        #self.execute_named_pose_sub = self.create_subscription(String, '/execute_named_pose', self.named_pose_topic_callback, 10) #Used by manager.py to move to specific poses based on pose name

        # Timers
        self.current_pose_timer = self.create_timer(0.1, self.current_pose_timer_callback, callback_group=self.parallel_group) #0.1s = 10Hz for DMP get_eef_pose

        # Service servers
        self.check_button_state_srv = self.create_service(
            CheckButtonState,
            'check_button_state',
            self.check_button_state_callback
        )
        self.get_logger().info("Check button state service created")

        # self.get_position_srv = self.create_service(
        #     GetPosition,
        #     'get_position',
        #     self.get_position_callback
        # )
        # self.get_logger().info("Get position service created")

        self.set_arm_mode_srv = self.create_service(
            SetArmMode,
            'set_arm_mode',
            self.set_arm_mode_callback
        )
        self.get_logger().info("Set arm mode service created")

        self.impedance_control_srv = self.create_service(
            ImpedanceControl,
            'impedance_control',
            self.impedance_control_callback
        )
        self.get_logger().info("Impedance control service created")

        # Action Servers
        self.move_to_pose_action_server = ActionServer(
            self,
            MoveToPose,
            'move_to_pose',
            self.pose_action_callback,
            callback_group=self.parallel_group
        )
        self.get_logger().info("Move to pose action server created")

        # Initialisation of states
        self.is_paused = False
        self.setup_arm(True)
        # Turn off LED initially
        self.arm.set_cgpio_analog(1, 0.0)

    # ============================================================================
    # GENERAL xArm FUNCTIONS
    # ============================================================================

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
        self.arm.clean_error()
        self.arm.set_mode(0)
        self.arm.set_state(0)
        time.sleep(0.1)

        self._ft_sensor_enabled = False
        self._impedance_control_enabled = False

        if reset:
            # self.named_pose_topic_callback(String(data="reset"))
            print("Moving to Reset Pose...")
            print("state before moving to reset: ", self.arm.get_state())
            ret = self.arm.set_position(*self._reset_pose, 
                            speed=self.default_speed, 
                            mvacc=self.default_accel, 
                            wait=True)
            if ret != 0:
                print("Failed to set position, error code:", ret)
            else:
                print("Move to reset position successfully")
            self.arm.set_state(0)
            time.sleep(0.1)


    def disconnect_arm(self, reset=False):
        """
        Safely disconnects the arm and resets if required.

        Args:
            arm (XArmAPI): Arm class
            reset (bool, optional): Whether to reset robot to initial position (joint pos = 0). Defaults to False.
        """  
        # Reset back to position mode
        self.arm.set_mode(0)
        self.arm.set_state(0)
        
        if self._impedance_control_enabled:
            self.disable_impedance_control()
        
        if self._ft_sensor_enabled:
            self.arm.ft_sensor_enable(0)

        if reset:
            self.arm.reset(wait=True)

        self.arm.set_cgpio_analog(1, 0.0)  # Turn off LED
        
        self.arm.disconnect()

    def set_pose_callback(self, msg):
        """
        Move the robot to a given pose
        pose: feeding_msgs/SetPosition
        """
        speed = msg.speed if msg.speed > 0 else self.default_speed
        mvacc = msg.mvacc if msg.mvacc > 0 else self.default_accel
        is_wait = msg.is_wait
        motion_type = msg.motion_type

        if self.is_paused:
            self.get_logger().warn("Ignoring new pose command because arm is currently paused")
            self.arm.set_state(3) # TODO fix the proper pause behavior
        
        self.arm.set_state(0)
        current_state = self.arm.get_state()
        # self.get_logger().info(f"xArm current state code: {current_state}")
        
        ret = self.arm.set_position(
                x=msg.target_point.x, y=msg.target_point.y, z=msg.target_point.z,
                roll=msg.roll, pitch=msg.pitch, yaw=msg.yaw,
                speed=speed, 
                mvacc=mvacc, 
                wait=is_wait,
                motion_type=motion_type
            )

        # if ret != 0:
        #     print("Failed to set position, error code:", ret)
        # else:
        #     self.setPosCount+=1
        #     print("setPosCount", self.setPosCount)
        #     print("Move to position successfully", msg.target_point.x, msg.target_point.y, msg.target_point.z, msg.roll, msg.pitch, msg.yaw)

        if ret != 0:
            self.get_logger().error(f"Failed to set position (error code: {ret})")
        else:
            self.setPosCount += 1
            self.get_logger().info(
                f"[setPosCount={self.setPosCount}] Move success → "
                f"pos=({msg.target_point.x:.3f}, {msg.target_point.y:.3f}, {msg.target_point.z:.3f}), "
                f"rpy=({msg.roll:.2f}, {msg.pitch:.2f}, {msg.yaw:.2f})"
        )

    #TODO: Remove this old topic-based callback and use only the action server
    # def named_pose_topic_callback(self, msg):
    #     """
    #     Move the robot to a given pose given the name of the pose
    #     pose: std_msgs/String
    #     """
    #     pose = msg.data
    #     if self.is_paused:
    #         self.get_logger().warn("Ignoring new pose command because arm is currently paused")
        
    #     self.arm.set_state(0)
    #     current_state = self.arm.get_state()
    #     self.get_logger().info(f"xArm current state code: {current_state}")
        
    #     if pose == "perception":
    #         self.get_logger().info("Moving to Perception Pose...")
    #         ret = self.arm.set_position(*self._perception_pose, 
    #                           speed=self.default_speed, 
    #                           mvacc=self.default_accel, 
    #                           radius=0, wait=True)
    #     elif pose == "transfer":
    #         self.get_logger().info("Moving to Bite Transfer Start Pose...")
    #         ret = self.arm.set_position(*self._transfer_pose, 
    #                           speed=self.default_speed, 
    #                           mvacc=self.default_accel, 
    #                           radius=0, wait=True)
    #     elif pose == "reset":
    #         self.get_logger().info("Moving to Reset Pose...")
    #         ret = self.arm.set_position(*self._reset_pose, 
    #                         speed=self.default_speed, 
    #                         mvacc=self.default_accel, 
    #                         radius=0, wait=True)
    #     else:
    #         self.get_logger().warn("Invalid Pose name")
    #         pass

    #     if ret != 0:
    #         print("Failed to set position, error code:", ret)
    #     else:
    #         print("Move to pose successfully")

    def pose_action_callback(self, goal_handle):
        """
        Action Server callback to move the robot to either a named pose
        or set position and quartenions
        """
        # 1. Safety/Pause Check 
        if self.is_paused:
            self.get_logger().warn("Aborting goal: Arm is currently paused")
            goal_handle.abort()
            return MoveToPose.Result(success=False, message="Arm is paused")
        
        # 2. Prepare Request Data
        req = goal_handle.request
        speed = req.speed if req.speed > 0 else self.default_speed
        mvacc = req.mvacc if req.mvacc > 0 else self.default_accel
        motion_type = req.motion_type
        
        # 3. Handle Movement Execution
        self.arm.set_state(0)

        # Define movement parameters based on named vs specific pose
        if req.named_pose:
            pose_map = {
                "perception": self._perception_pose,
                "transfer": self._transfer_pose,
                "reset": self._reset_pose
            }
            
            if req.pose_name not in pose_map:
                self.get_logger().error(f"Invalid Pose name: {req.pose_name}")
                goal_handle.abort()
                return MoveToPose.Result(success=False, message=f"Unknown pose: {req.pose_name}")
            
            target = pose_map[req.pose_name]
            self.get_logger().info(f"Moving to named pose: {req.pose_name}")

            self.arm.motion_enable(enable=True)
            # Execution (Non-blocking so we can check for cancellations)
            ret = self.arm.set_position(*target, speed=speed, mvacc=mvacc, wait=True)
        
        else:
            self.get_logger().info("Using pose defined by position and quarternion for action server")
            ret = -1
            req = goal_handle.request

            self.get_logger().info(
                "Moving to target pose: "
                f"pos=({req.target_point.x:.3f}, {req.target_point.y:.3f}, {req.target_point.z:.3f}), "
                f"rpy=({req.roll:.2f}, {req.pitch:.2f}, {req.yaw:.2f})"
            )
            ret = self.arm.set_position(
                x=req.target_point.x, y=req.target_point.y, z=req.target_point.z,
                roll=req.roll, pitch=req.pitch, yaw=req.yaw,
                speed=speed, 
                mvacc=mvacc, 
                wait=True,
                motion_type=motion_type
            )
    
        # 4. Handle Success/Failure
        result = MoveToPose.Result()
        
        if ret == 0: # 0 usually means success in xArm API
            self.get_logger().info("Action Succeeded: Move complete")
            goal_handle.succeed()
            result.success = True
            result.message = "Movement successful"
        else:
            self.get_logger().error(f"Action Aborted: xArm error code {ret}")
            goal_handle.abort()
            result.success = False
            result.message = f"xArm API Error: {ret}"

        return result

    def pause_callback(self, msg):
        """
        Callback to handle stop/paused requests
        """
        paused = msg.data
        if paused is True and not self.is_paused:
            self.get_logger().warn("PAUSE COMMAND RECEIVED: Stopping the arm immediately.")
            self.arm.set_state(3)
            self.is_paused=True

        elif paused is False and self.is_paused:
            self.get_logger().info("RESUME COMMAND RECEIVED: Enabling Motion.")
            self.arm.set_state(0)

        elif paused is True and self.is_paused:
            self.get_logger().info("ALREADY PAUSED: Doing Nothing")
        
        else:
            pass

    def set_led_callback(self, msg):
        led_state = msg.data
        if led_state == True:
            ret = self.arm.set_cgpio_analog(1, 8.0) #ON state
        elif led_state == False:
            ret = self.arm.set_cgpio_analog(1, 0.0) #OFF state
        else:
            pass
        return ret
    
    def check_button_state_callback(self, request, response):
        """
        Service callback to check the button state.
        Returns the C14 GPIO pin state (0 = pressed, 1 = not pressed).
        """
        try:
            if request.check_state:
                digital_inputs = self.arm.get_cgpio_digital()
                # digital_inputs[1][0] is the C14 pin state
                c14_state = digital_inputs[1][0]
                response.button_state = c14_state
                response.success = True
                self.get_logger().debug(f"Button state: {c14_state}")
            else:
                response.button_state = -1
                response.success = False
        except Exception as e:
            self.get_logger().error(f"Failed to check button state: {str(e)}")
            response.button_state = -1
            response.success = False
        
        return response
    
    def current_pose_timer_callback(self):
        """
        Timer callback to publish the current end-effector position and orientation at regular intervals.
        """
        try:
            pose = self.arm.get_position()
            msg = GetPosition()
            msg.position.x = pose[1][0]
            msg.position.y = pose[1][1]
            msg.position.z = pose[1][2]
            msg.roll = pose[1][3]
            msg.pitch = pose[1][4]
            msg.yaw = pose[1][5]
            self._current_pose_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish current pose: {str(e)}")
            self.arm.set_state(3)  # Set to paused state on failure
            self.is_paused = True

    # def get_position_callback(self, request, response):
    #     """
    #     Service callback to get the current end-effector position and orientation.
    #     """
    #     try:
    #         pose = self.arm.get_position()
    #         response.target_point.x = pose[1][0]
    #         response.target_point.y = pose[1][1]
    #         response.target_point.z = pose[1][2]
    #         response.roll = pose[1][3]
    #         response.pitch = pose[1][4]
    #         response.yaw = pose[1][5]
    #         ret = pose[0]
    #         response.success = (ret == 0)
    #         if not response.success:
    #             self.get_logger().error(f"Failed to get position, error code: {ret}")
    #         self.get_logger().debug(f"Current position: {pose[1]}")
    #     except Exception as e:
    #         self.get_logger().error(f"Failed to get position: {str(e)}")
    #         response.success = False
    #         self.arm.set_state(3)  # Set to paused state on failure
    #         self.is_paused = True
        
    #     return response

    def set_arm_mode_callback(self, request, response):
        """
        Service callback to set the arm mode.
        Mode 0: Position control
        Mode 7: Impedance control
        """
        try:
            ret = self.arm.set_mode(request.mode)
            if ret == 0:
                self.arm.set_state(0)
                time.sleep(0.1)
                response.success = True
                self.get_logger().info(f"Arm mode set to {request.mode}")
            else:
                response.success = False
                self.get_logger().error(f"Failed to set arm mode, error code: {ret}")
        except Exception as e:
            self.get_logger().error(f"Failed to set arm mode: {str(e)}")
            response.success = False
        
        return response

    def impedance_control_callback(self, request, response):
        """
        Service callback to enable or disable impedance control.
        """
        try:
            if request.enable:
                if not self._impedance_control_enabled:
                    self.setup_arm_impedance_control(enable=True)
                    response.success = True
                    self.get_logger().info("Impedance control enabled")
                else:
                    response.success = True
                    self.get_logger().info("Impedance control already enabled")
            else:
                if self._impedance_control_enabled:
                    self.disable_impedance_control()
                    response.success = True
                    self.get_logger().info("Impedance control disabled")
                else:
                    response.success = True
                    self.get_logger().info("Impedance control already disabled")
        except Exception as e:
            self.get_logger().error(f"Failed to toggle impedance control: {str(e)}")
            response.success = False
        
        return response

    # ============================================================================
    # FT SENSOR FUNCTIONS
    # ============================================================================
        
    def setup_arm_impedance_control(self, enable=True):
        """
        Set up the xArm robot in impedance control mode.
        """
        # Set tool impedance parameters:
        K_POS = 150         #  x/y/z linear stiffness coefficient, range: 0 ~ 2000 (N/m)
        K_ORI = 4           #  Rx/Ry/Rz rotational stiffness coefficient, range: 0 ~ 20 (Nm/rad)

        # Attention: for M and J, smaller value means less effort to drive the arm, but may also be less stable, please be careful. 
        M = float(0.06)  #  x/y/z equivalent mass; range: 0.02 ~ 1 kg
        J = M * 0.01     #  Rx/Ry/Rz equivalent moment of inertia, range: 1e-4 ~ 0.01 (Kg*m^2)

        C_AXIS = [0,1,1,0,0,0]  # set z axis as compliant axis
        REF_FRAME = 0           # 0 : base , 1 : tool

        self.arm.set_impedance_mbk([M, M, M, J, J, J], [K_POS, K_POS, K_POS, K_ORI, K_ORI, K_ORI], [0]*6) # B(damping) is reserved, give zeros
        self.arm.set_impedance_config(REF_FRAME, C_AXIS)
        
        # Enable ft sensor communication
        self.arm.ft_sensor_enable(1)
        self._ft_sensor_enabled = True

        if enable:
            self.enable_impedance_control()
            print("Impedance control setup and enabled.")
        time.sleep(0.1)
    
    def enable_impedance_control(self):
        self.arm.ft_sensor_app_set(1)
        self.arm.set_state(0)        # Will start impedance mode after set_state(0)
        self._impedance_control_enabled = True
        self.get_logger().warn("Enabled impedance control")

    def disable_impedance_control(self):
        self.arm.ft_sensor_app_set(0)
        self.arm.set_state(0)       
        self._impedance_control_enabled = False
        print("Disabled impedance control")

# ============================================================================
# MAIN
# ============================================================================

def main(args=None):
    rclpy.init(args=args)    
    ac = ArmController()

    executor = MultiThreadedExecutor()
    executor.add_node(ac)

    try:
        executor.spin()
    except KeyboardInterrupt:
        ac.get_logger().info("Closing arm_controller")
        ac.disconnect_arm()
    finally:
        executor.shutdown()
        ac.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
