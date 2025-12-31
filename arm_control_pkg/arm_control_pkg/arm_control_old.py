#!/usr/bin/env python
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.executors import SingleThreadedExecutor
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool, String, Empty
from geometry_msgs.msg import PointStamped
from std_srvs.srv import SetBool
from feeding_msgs.action import BiteTransfer
from feeding_msgs.msg import RobotFeedback
from feeding_msgs.srv import CheckButtonState

from gtts import gTTS
from pydub import AudioSegment
from pydub.playback import play
from bite_transfer.ft_sensor_spoon import M8128TCPClient
from bite_transfer.sri_sensor_recorder import SensorRecorder

from scipy.spatial.transform import Rotation as R
from xarm.wrapper import XArmAPI

import quaternion
import warnings

from task_planner.text_to_speech import say
warnings.filterwarnings("ignore", message=".*ALSA lib.*")

"""
Handles the physical xArm6 robot arm during the feeding sequence.

Organized into sections:
1. Initialization & ROS Setup
2. General xArm Functions
3. Bite Transfer Functions
4. FT Sensor Functions
5. Utility & Transform Functions
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
        
        ################################################
        # Bite Transfer Related Parameters
        ################################################

        # Mouth tracking params
        self.margin_of_error = 20  # Define a margin of error
        self.mouth_closed_offset = 200

        # Sets control frequency for admittance control
        self.control_freq = 200
        self.dt = 1.0 / self.control_freq

        # Initialize bite transfer params
        self.distance_to_mouth = 2.0
        self.exit_angle = 100
        self.entry_angle = 90
        self.height_offset = 0

        # Initialise initial readings for force-torque sensor
        self._initial_reading = None
        self._check_bite_initial_reading = None
        self.point_x = self.point_y = self.point_z = None
        self._bite_detected = False
        self.mouth_status = "Unknown"
        self.latest_mouth_point = None
        self._feedback_given = False

        self.declare_parameter("bite_threshold", 3.0)
        self._bite_threshold = self.get_parameter("bite_threshold").value  # Default threshold for bite detection
        self.declare_parameter("spoon_length", 0.195)
        self._spoon_length = self.get_parameter("spoon_length").value

        self._min_transfer_speed = 30
        self._max_transfer_speed = 300
        self._mvacc = 100
        self._mvacc_mouth = 100
        self._x_max = 720
        self._x_min = 500
        self._z_max = 450
        self._impedance_control = True

        # Publishers:
        self.robot_feedback_pub = self.create_publisher(RobotFeedback, "/robot_feedback", 1)

        # Subscribers
        self.transformed_point_sub = self.create_subscription(PointStamped,'/transformed_point', self.point_callback, 1)
        self.mouth_status_sub = self.create_subscription(String, '/mouth_status', self.mouth_status_callback, 1)
        
        self.sub_target_pose = self.create_subscription(String, '/target_pose', self.pose_callback, 10)
        self.sub_pause_cmd = self.create_subscription(Bool, '/pause_cmd', self.pause_callback, 1)
        self.sub_led_state = self.create_subscription(Bool, '/led_state', self.set_led_callback, 1)

        # Service servers
        self.check_button_state_srv = self.create_service(
            CheckButtonState,
            'check_button_state',
            self.check_button_state_callback
        )
        self.get_logger().info("Check button state service created")

        # Service clients
        self._mouth_tracking_toggle_srv = self.create_client(SetBool, '/toggle_mouth_tracking')
        while not self._mouth_tracking_toggle_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Mouth tracking toggle service not available, waiting...')

        # Bite Transfer Action Server
        self.server = ActionServer(
            self,
            BiteTransfer,
            'bite_transfer',
            execute_callback=self.execute_bite_transfer_cb,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.is_paused = False
        self.setup_arm(True)
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
        self.arm.set_mode(0)
        self.arm.set_state(0)
        time.sleep(0.1)

        self._ft_sensor_enabled = False
        self._impedance_control_enabled = False

        if reset:
            self.pose_callback(String(data="reset"))

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
        
        self.arm.disconnect()

    def pose_callback(self, msg):
        """
        Move the robot to a given pose given the name of the pose
        pose: std_msgs/String
        """
        pose = msg.data
        if self.is_paused:
            self.get_logger().warn("Ignoring new pose command because arm is currently paused")
        
        self.arm.set_state(0)
        current_state = self.arm.get_state()
        self.get_logger().info(f"xArm current state code: {current_state}")
        
        if pose == "perception":
            self.get_logger().info("Moving to Perception Pose...")
            ret = self.arm.set_position(*self._perception_pose, 
                              speed=self.default_speed, 
                              mvacc=self.default_accel, 
                              radius=0, wait=True)
        elif pose == "transfer":
            self.get_logger().info("Moving to Bite Transfer Start Pose...")
            ret = self.arm.set_position(*self._transfer_pose, 
                              speed=self.default_speed, 
                              mvacc=self.default_accel, 
                              radius=0, wait=True)
        elif pose == "reset":
            self.get_logger().info("Moving to Reset Pose...")
            ret = self.arm.set_position(*self._reset_pose, 
                            speed=self.default_speed, 
                            mvacc=self.default_accel, 
                            radius=0, wait=True)
        else:
            self.get_logger().warn("Invalid Pose name")
            pass

        if ret != 0:
            print("Failed to set position, error code:", ret)
        else:
            print("Move to pose successfully")

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

    # ============================================================================
    # BITE TRANSFER FUNCTIONS
    # ============================================================================

    def goal_callback(self, goal_request):
        self.get_logger().info('Received bite transfer goal request')
        return GoalResponse.ACCEPT  
    
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received request to cancel bite transfer goal')
        return CancelResponse.ACCEPT

    def execute_bite_transfer_cb(self, goal_handle):
        """Execute callback for the action server (continuous mouth tracking)."""
        
        feedback = BiteTransfer.Feedback()
        result = BiteTransfer.Result()

        try:
            goal = goal_handle.request

            self.distance_to_mouth = goal.distance_to_mouth
            self.exit_angle = goal.exit_angle
            alpha = (goal.transfer_speed - 1) / 4
            self.transfer_speed = self._min_transfer_speed * ((self._max_transfer_speed / self._min_transfer_speed) ** alpha)
            self.transfer_speed = np.clip(self.transfer_speed, self._min_transfer_speed, self._max_transfer_speed)
            self.height_offset = goal.height_offset
            if goal.entry_angle != 0:
                self.entry_angle = goal.entry_angle

            self.get_logger().info(f"Start signal received. Beginning bite transfer at {self.transfer_speed:.2f} mm/s")
            feedback.status = "Bite transfer started."
            goal_handle.publish_feedback(feedback)

            self.setup_sri_ft_sensor()
            self.setup_arm_impedance_control(enable=True) # Check if we need this

            req = SetBool.Request()
            req.data = True
            future = self._mouth_tracking_toggle_srv.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            resp = future.result()

            self.move_to_start()
            self.arm.set_mode(7)
            self.arm.set_state(0)
            time.sleep(0.1)

            ft_data = np.array(self._sri_client.get_curr_data())
            print(ft_data)
            if self._initial_reading is None:
                self._initial_reading = ft_data.copy()
                print("success")
            self._check_bite_initial_reading = ft_data - self._initial_reading
            self.get_logger().warn(f"Initial FT reading: {self._initial_reading[0:3]}")

            self.give_feedback(message="I will feed you now", level="info")
            self._feedback_given = False
            self._bite_detected = False
            
            self.wait_for_point()

            if self._impedance_control:
                ret = self.impedance_move_to_point()
            else:
                ret = self.admittance_move_to_point()

            if ret:
                self.get_logger().info("Mouth servoing completed. Mouth point reached or bite detected.")
            else:
                self.get_logger().warn("Robot failed to reach mouth point")
                raise Exception("Failed to reach mouth point")
            
            self.arm.set_mode(0)
            self.arm.set_state(0)  

            if not self._bite_detected:
                self.give_feedback(message="You may take a bite now", level="info")
                self.get_logger().info("Checking for bite")

                self._check_bite_initial_reading = None
                while rclpy.ok():                
                    if self.check_for_bite(disable_impedance=True):
                        self._check_bite_initial_reading = None                        
                        break
            else:
                self.get_logger().info("Bite detected before reaching mouth point")
            
            self.move_to_exit_point()
            self.get_logger().info("Successfully moved to exit point")
            self.move_to_start()

            result.success = True
            goal_handle.succeed()
            feedback.status = "Bite transfer completed successfully"
            goal_handle.publish_feedback(feedback)
            return result

        except Exception as e:
            self.get_logger().error(f"Bite transfer failed: {e}")
            feedback.status = f"Error: {e}"
            result.success = False
            goal_handle.abort()
            return result

        finally:
            try:
                self._bite_detected = False
                req = SetBool.Request()
                req.data = False
                future = self._mouth_tracking_toggle_srv.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                resp = future.result()

                if hasattr(self, 'sensor_recorder'):
                    self.sensor_recorder.stop()

                self.disconnect_sri_ft_sensor()
                time.sleep(0.1)
                self.get_logger().info("Completed bite transfer, cleaned up.")
            except Exception as e:
                self.get_logger().error(f"Error during cleanup: {e}")

    def give_feedback(self, message, level, audio=True):
        """Publishes feedback to the robot_feedback topic for the WebApp and optionally plays audio.
        Args:
            message (str): Feedback message to be published.
            level (str): Level of feedback (info, warn, error).
            audio (bool, optional): Whether to play audio. Defaults to True.
        """
        self.robot_feedback_pub.publish(RobotFeedback(feedback=message, level=level))
        if audio:
            audio_thread = say(message)
            if audio_thread is not None:
                audio_thread.join()

    def point_callback(self, msg):
        """
        Extracts the x, y, z values of the mouth pose from the /transformed_point topic.

        Args:
            msg (PointStamped): PointStamped message containing x, y, z values.
        """            
        self.point_x, self.point_y, self.point_z = msg.point.x, msg.point.y, msg.point.z
        self.latest_mouth_point = msg.point

    def mouth_status_callback(self, msg):
        """Determines if the mouth is closed or opened from the /mouth_status topic.

        Args:
            msg (String): String message containing mouth status.
        """
        self.mouth_status = msg.data

    def wait_for_point(self):
        """
        Waits for a point to be received (ie mouth is detected)
        """
        rate = self.create_rate(10)
        while self.point_x is None and rclpy.ok():
            self.get_logger().warn("Waiting for point...")
            if not self._feedback_given:
                self.give_feedback("Mouth not detected! Please ensure you are in the frame", level="warn")
                self._feedback_given = True
            rate.sleep()
        return None

    def move_to_entry_point(self):
        """Moves to the entry point.
        """         
        current_position = self.arm.get_position()[1]
        print("Current EEF pose (rpy):", current_position)

        entry_angle = self.entry_angle
        spoon_offset = np.array([0, 0, self._spoon_length*1000])

        final_pose_rpy = self.calculate_entry_exit_pose(current_position, entry_angle, spoon_offset)
        print("Final EEF pose (rpy):", final_pose_rpy)
                
        self.arm.set_position(
            x=final_pose_rpy[0],
            y=final_pose_rpy[1],
            z=final_pose_rpy[2] + self.height_offset,
            roll=final_pose_rpy[3],
            pitch=final_pose_rpy[4],
            yaw=final_pose_rpy[5],
            speed=self.transfer_speed,
            mvacc=self._mvacc_mouth,
            radius=0,
            wait=False
        )
        time.sleep(1)
        print("Moved to entry point")

    def move_to_exit_point(self):
        """
        Command the robot to move to the exit point after a bite is detected.
        """
        if self._impedance_control:
            if not self._impedance_control_enabled:
                self.enable_impedance_control()

        current_position = self.arm.get_position()[1]
        print("Current EEF pose (rpy):", current_position)
        
        exit_angle = self.exit_angle
        spoon_offset = np.array([0, 0, self._spoon_length*1000])

        final_pose_rpy = self.calculate_entry_exit_pose(current_position, exit_angle, spoon_offset)
        print("Final EEF pose (rpy):", final_pose_rpy)
        
        self.arm.set_position(
            x=final_pose_rpy[0] - 40,
            y=final_pose_rpy[1],
            z=final_pose_rpy[2],
            roll=final_pose_rpy[3],
            pitch=final_pose_rpy[4],
            yaw=final_pose_rpy[5],
            speed=self.transfer_speed,
            mvacc=self._mvacc_mouth,
            radius=0,
            wait=True
        )

    def move_to_start(self, wait = True):
        """Moves to start position before commencing feeding.

        Args:
            wait (bool, optional): whether to wait for the arm to reach position. Defaults to True.
        """
        arm_start_pose = self._transfer_pose
        start_t = time.time()
        self.arm.set_position(x = arm_start_pose[0], 
                              y = arm_start_pose[1], 
                              z = arm_start_pose[2], 
                              roll = arm_start_pose[3], 
                              pitch = arm_start_pose[4], 
                              yaw = arm_start_pose[5], 
                              speed = self.transfer_speed, 
                              mvacc = self._mvacc, 
                              radius = 0, 
                              wait = True)
        
        print(f"Moved to start position in {time.time() - start_t:.2f} seconds")

    def impedance_move_to_point(self):
        """
        Moves the robot to a target point using impedance control. 
        """
        margin_of_error_time = 1.0
        last_time_in_margin = None
        curr_mouth_target_pose = None
        mouth_undetected_start_t = None
        mouth_too_far_start_t = None
        mouth_far_warn_time = None
        mouth_too_close_start_t = None
        mouth_close_warn_time = None
        mouth_detected = False

        try:
            while rclpy.ok():
                rclpy.spin_once(self)
                base_x, base_y, base_z = self.point_x, self.point_y, self.point_z
                quat = np.array(self.rpy_to_quaternion(3.141, -1.57, 0))
                points = [base_x, base_y, base_z, quat[3], quat[0], quat[1], quat[2]]
                points_world = self.get_eef_pose_from_spoon_pose(points, np.array([0, 0, self._spoon_length]))

                base_x, base_y, base_z = points_world[0]*1000, points_world[1]*1000, points_world[2]*1000

                if self.mouth_status in ["Open", "Closed"]:
                    curr_mouth_target_pose = np.array([base_x- (self.distance_to_mouth*10), base_y, base_z, 3.141, -1.57, 0])
                    
                    if curr_mouth_target_pose[0] > self._x_max:
                        if mouth_too_far_start_t is None:
                            self.get_logger().warn("Mouth detected but too far away... ")
                            mouth_too_far_start_t = time.time()
                                
                        if (mouth_far_warn_time is None) or (time.time() - mouth_far_warn_time > 5.0):   
                            self.give_feedback("You are too far away! Please move closer", level="warn")
                            mouth_far_warn_time = time.time()

                        mouth_detected = False
                        curr_mouth_target_pose[0] = self._x_max - 50
                    
                    elif curr_mouth_target_pose[0] < self._x_min:
                        if mouth_too_close_start_t is None:
                            self.get_logger().warn("Mouth detected but too close... ")
                            mouth_too_close_start_t = time.time()
                                
                        if (mouth_close_warn_time is None) or (time.time() - mouth_close_warn_time > 5.0):   
                            self.give_feedback("You are too close to me! Please move back", level="warn")
                            mouth_close_warn_time = time.time()

                        curr_mouth_target_pose[0] = self._x_min
                        mouth_detected = False
                    
                    elif curr_mouth_target_pose[2] > self._z_max:
                        self.get_logger().warn("3ddfa likely hallucinating with very high z")
                        mouth_detected = False
                        curr_mouth_target_pose[2] = self._z_max

                    else:
                        mouth_detected = True
                        mouth_undetected_start_t = None
                        mouth_too_far_start_t = None
                        mouth_too_close_start_t = None

                else:
                    mouth_detected = False
                    curr_mouth_target_pose = self.arm.get_position()[1]
                    if mouth_undetected_start_t is None:
                        self.get_logger().warn("Mouth not detected. Waiting for mouth to be detected...")
                        self.give_feedback("Mouth not detected! Please ensure you are in the frame", level="warn")
                        mouth_undetected_start_t = time.time()
                    elif time.time() - mouth_undetected_start_t >= 30.0:
                        self.get_logger().warn("Mouth not detected for too long. Stopping movement.")
                        self.give_feedback("Mouth not detected for too long! Exiting...", level="warn")
                        return False

                target_pose = curr_mouth_target_pose.copy()
                print("Moving to target pose:", target_pose)
                ret = self.arm.set_position(
                        x=target_pose[0], y=target_pose[1], z=target_pose[2],
                        roll=target_pose[3], pitch=target_pose[4], yaw=target_pose[5],
                        speed=self.transfer_speed, 
                        mvacc=self._mvacc_mouth,
                        wait=False
                    )
                
                if ret != 0:
                    print("Error occurred while moving the robot.")
                    return False
                
                if self.check_for_bite():
                    self._bite_detected = True
                    self._check_bite_initial_reading = None
                    return True

                if mouth_detected:
                    current_pose = self.arm.get_position()[1]

                    within_margin = all(
                        abs(current_pose[i] - target_pose[i]) < self.margin_of_error for i in range(3)
                    )
                    if within_margin:
                        if last_time_in_margin is None:
                            last_time_in_margin = time.time()
                        elif time.time() - last_time_in_margin >= margin_of_error_time:
                            self.get_logger().info("Target point reached and stable. Impedance control loop finished.")
                            return True
                    else:
                        last_time_in_margin = None

        except KeyboardInterrupt:
            self.get_logger().warn("Exiting impedance control loop due to interrupt.")

    # UNUSED: Admittance control is not currently used, impedance control is preferred
    def admittance_controller(self, ft_data):
        """Applies admittance control to adjust the target pose based on force-torque sensor data. 

        Args:
            ft_data (np.array): Force-torque sensor data.
        
        Returns:
            np.array: Delta position and orientation based on admittance control.
        """        

        M = np.diag([0.05, 0.05, 0.05, 0.01, 0.01, 0.01])  
        D = np.diag([6.0, 6.0,6.0, 1.0, 1.0, 1.0])  

        ft_data = np.array([ft_data[2], -ft_data[1], ft_data[0], ft_data[5], -ft_data[4], ft_data[3]])

        gain =  np.diag([14.0, 20.0, 25.0, 1.5, 1.5, 1.5])  
        ft_data = gain @ ft_data

        joint_states = self.arm.get_joint_states()[1]
        joint_velocities = joint_states[1]

        linear_vel = joint_velocities[:3]
        angular_vel = joint_velocities[3:6]

        vel = np.hstack((linear_vel, angular_vel))
        acc = np.linalg.inv(M) @ (ft_data - D @ vel)

        linear_vel_new = (linear_vel + acc[:3] * self.dt)*1000
        angular_vel_new = (angular_vel + acc[3:] * self.dt) 

        delta_pos = linear_vel_new * self.dt
        delta_ori = angular_vel_new * self.dt

        return delta_pos, delta_ori

    # UNUSED: Admittance control is not currently used, impedance control is preferred
    def admittance_move_to_point(self):
        """
        Moves the robot to a target point using admittance control. 
        Adjusts positions based off admittance_controller delta pos and ori.
        """
        margin_of_error_time = 1.0
        last_time_in_margin = None
        curr_mouth_target_pose = None
        mouth_undetected_start_t = None
        mouth_too_far_start_t = None
        mouth_far_warn_time = None
        mouth_too_close_start_t = None
        mouth_close_warn_time = None
        mouth_detected = False

        try:
            while rclpy.ok():
                base_x, base_y, base_z = self.point_x, self.point_y, self.point_z
                quat = np.array(self.rpy_to_quaternion(-3.141, -1.571, 0))
                points = [base_x, base_y, base_z, quat[3], quat[0], quat[1], quat[2]]
                points_world = self.get_eef_pose_from_spoon_pose(points, np.array([0, 0, self._spoon_length]))

                base_x, base_y, base_z = points_world[0]*1000, points_world[1]*1000, points_world[2]*1000

                if self.mouth_status in ["Open", "Closed"]:
                    curr_mouth_target_pose = np.array([base_x- (self.distance_to_mouth*10), base_y, base_z, -3.141, -1.4, 0])
                    if curr_mouth_target_pose[0] > self._x_max:
                        if mouth_too_far_start_t is None:
                            self.get_logger().warn("Mouth detected but too far away... ")
                            mouth_too_far_start_t = time.time()
                                
                        if (mouth_far_warn_time is None) or (time.time() - mouth_far_warn_time > 5.0):   
                            self.give_feedback("You are too far away! Please move closer", level="warn")
                            mouth_far_warn_time = time.time()

                        mouth_detected = False
                        curr_mouth_target_pose[0] = self._x_max - 50
                    
                    elif curr_mouth_target_pose[0] < self._x_min:
                        if mouth_too_close_start_t is None:
                            self.get_logger().warn("Mouth detected but too close... ")
                            mouth_too_close_start_t = time.time()
                                
                        if (mouth_close_warn_time is None) or (time.time() - mouth_close_warn_time > 5.0):   
                            self.give_feedback("You are too close to me! Please move back", level="warn")
                            mouth_close_warn_time = time.time()

                        curr_mouth_target_pose[0] = self._x_min
                        mouth_detected = False

                    else:
                        mouth_detected = True
                        mouth_undetected_start_t = None
                        mouth_too_far_start_t = None
                        mouth_too_close_start_t = None

                else:
                    mouth_detected = False
                    curr_mouth_target_pose = self.arm.get_position()[1]
                    if mouth_undetected_start_t is None:
                        self.get_logger().warn("Mouth not detected. Waiting for mouth to be detected...")
                        self.give_feedback("Mouth not detected! Please ensure you are in the frame", level="warn")
                        mouth_undetected_start_t = time.time()
                    elif time.time() - mouth_undetected_start_t >= 30.0:
                        self.get_logger().warn("Mouth not detected for too long. Stopping movement.")
                        self.give_feedback("Mouth not detected for too long! Exiting...", level="warn")
                        return False

                target_pose = curr_mouth_target_pose.copy()

                ft_data = np.array(self._sri_client.get_curr_data())
                if self._initial_reading is None:
                    self._initial_reading = ft_data.copy()
                ft_data -= self._initial_reading

                if mouth_detected:
                    current_pose = self.arm.get_position()[1]

                    delta_pos, _ = self.admittance_controller(ft_data)

                    for i in range(3):
                        target_pose[i] += delta_pos[i]

                print("Moving to target pose:", target_pose)
                ret = self.arm.set_position(
                        x=target_pose[0], y=target_pose[1], z=target_pose[2],
                        roll=target_pose[3], pitch=target_pose[4], yaw=target_pose[5],
                        speed=self.transfer_speed, 
                        mvacc=self._mvacc_mouth,
                        wait=False
                    )
                
                if ret != 0:
                    print("Error occurred while moving the robot.")
                    return False
                
                if self.check_for_bite():
                    self._bite_detected = True
                    self._check_bite_initial_reading = None
                    return True

                if mouth_detected:
                    pose_delta = abs(current_pose[:3] - target_pose[:3])
                    print("POSE DELTA:", pose_delta)
                    within_margin = all(
                        pose_delta < self.margin_of_error
                    )
                    if within_margin:
                        if last_time_in_margin is None:
                            last_time_in_margin = time.time()
                        elif time.time() - last_time_in_margin >= margin_of_error_time:
                            self.get_logger().info("Target point reached and stable. Admittance control loop finished.")
                            return True
                    else:
                        last_time_in_margin = None

                time.sleep(self.dt)

        except KeyboardInterrupt:
            self.get_logger().warn("Exiting admittance control loop due to interrupt.")

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

    def setup_sri_ft_sensor(self, ip="192.168.1.108", port=4008):
        """
        Initialises the external SRI force-torque sensor

        Args:
            ip (str, optional): IP address of ft sensor. Defaults to "192.168.1.108".
            port (int, optional): Port number of ft sensor. Defaults to 4008.

        Raises:
            Exception: If connection to the sensor fails.
        """        
        self._sri_client = M8128TCPClient(ip, port, sampling_rate=200)

        if not self._sri_client._is_connected:
            raise Exception("Failed to connect to SRI FT sensor")
    
    def disconnect_sri_ft_sensor(self):
        """
        Disconnects the external_force_torque sensor.
        """
        self._sri_client.close_tcp()

    def check_for_bite(self, disable_impedance=False):
        """Check for a bite based on force-torque sensor data.

        Args:
            ft_data (np.array): Force-torque sensor data.

        Returns:
            bool: True if a bite is detected, False otherwise.
        """     
        # Need to disable impedance control here to prevent drift
        if disable_impedance:
            if self._impedance_control_enabled:
                self.disable_impedance_control()
                time.sleep(0.1)

        try:   
            ft_data = np.array(self._sri_client.get_curr_data())
            ft_data -= self._initial_reading

            if self._check_bite_initial_reading is None:
                self._check_bite_initial_reading = ft_data.copy()

            delta_ft = self._check_bite_initial_reading - ft_data

            # print(f" FT data: {ft_data[0:3]}")
            # print(f"Delta FT data: {delta_ft[0:3]}")

            # if too easy to detect, can also make sure that delta_ft[2] > 1.0
            if delta_ft[2] > self._bite_threshold: # and delta_ft[2] > 0.7:
                print("Bite detected with delta:", delta_ft[0:3])
                return True
            else:
                return False
        except KeyboardInterrupt:
            print("Exiting bite detection loop due to interrupt.")
            return False
        except Exception as e:
            print(f"Error occurred: {e}")
            return False

    # ============================================================================
    # UTILITY & TRANSFORM FUNCTIONS
    # ============================================================================

    def get_eef_pose_from_spoon_pose(self, spoon_pose, spoon_offset):
        """
        Removes the spoon offset from the spoon trajectory to get the eef trajectory

        Args:
            spoon_pose (ndarray): Numpy array of the spoon pose in [x, y, z, w, x, y, z]
            spoon_offset (ndarray): Offset to be applied to the spoon in [x, y, z]

        Returns:
            ndarray: Numpy array of the eef pose in [x, y, z, w, x, y, z]
        """
        spoon_pos = spoon_pose[: 3]
        spoon_quat = spoon_pose[3 :]
        spoon_quat = np.quaternion(spoon_quat[0], spoon_quat[1], spoon_quat[2], spoon_quat[3])
        spoon_rot_mat = quaternion.as_rotation_matrix(spoon_quat)

        eef_offset_world = spoon_rot_mat.dot(spoon_offset)
        eef_pos = spoon_pos - eef_offset_world

        eef_quat = spoon_quat

        eef_pose = np.hstack((eef_pos, eef_quat.components))

        return eef_pose

    def get_spoon_pose_from_eef_pose(self, eef_pose, spoon_offset):
        """
        Adds the spoon offset to the eef trajectory to get the spoon trajectory

        Args:
            eef_pose (ndarray): Numpy array of the end effector pose in [x, y, z, w, x, y, z]
            spoon_offset (ndarray): Offset to be applied to the eef pose in [x, y, z]

        Returns:
            ndarray: Numpy array of the spoon pose in [x, y, z, w, x, y, z]
        """
        spoon_pos = eef_pose[: 3] + spoon_offset

        eef_quat = eef_pose[3 :]
        eef_quat = np.quaternion(eef_quat[0], eef_quat[1], eef_quat[2], eef_quat[3])
        eef_rot_mat = quaternion.as_rotation_matrix(eef_quat)

        spoon_offset_world = eef_rot_mat.dot(spoon_offset)
        spoon_pos = eef_pose[: 3] + spoon_offset_world

        spoon_quat = eef_quat

        spoon_pose = np.hstack((spoon_pos, spoon_quat.components))
        return spoon_pose

    def normalize_angle(self, angle):
        """
        Normalize an angle to the range [-π, π].
        """
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def rpy_to_quaternion(self, roll, pitch, yaw):
        """
        Converts roll, pitch, yaw angles to a quaternion.
        """        
        r = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
        
        quaternion = r.as_quat()
        quaternion_wxyz = np.array([quaternion[3], quaternion[0], quaternion[1], quaternion[2]])
        return quaternion_wxyz

    def quaternion_to_rpy(self, quaternion_wxyz):
        """
        Converts a quaternion to roll, pitch, yaw angles.
        """
        quaternion_xyzw = np.array([quaternion_wxyz[1], quaternion_wxyz[2], quaternion_wxyz[3], quaternion_wxyz[0]])
        
        r = R.from_quat(quaternion_xyzw)
        roll, pitch, yaw = r.as_euler('xyz', degrees=False)
        
        roll = self.normalize_angle(roll)
        pitch = self.normalize_angle(pitch)
        yaw = self.normalize_angle(yaw)
        
        if np.isclose(roll, -np.pi) and np.isclose(yaw, 0, atol=1e-6):
            roll = 0
            yaw = -np.pi
        
        return np.array([roll, pitch, yaw])

    def calculate_entry_exit_pose(self, current_position, angle, spoon_offset):
        """
        Given the current EEF pose, an exit angle, and a spoon offset,
        compute the new EEF pose that rotates about the spoon tip.

        Args:
            current_position (ndarray): EEF pose in [x, y, z, roll, pitch, yaw].
            angle (float): Rotation angle (in degrees) to add to the spoon's pitch.
            spoon_offset (ndarray): Spoon offset in the EEF frame [x, y, z].

        Returns:
            ndarray: The final EEF pose in [x, y, z, roll, pitch, yaw].
        """
        current_position_quat = self.rpy_to_quaternion(
            current_position[3],
            current_position[4],
            current_position[5]
        )
        current_position_quat = np.concatenate((current_position[:3], current_position_quat))
        
        current_spoon_pose = self.get_spoon_pose_from_eef_pose(current_position_quat, spoon_offset)
        
        current_spoon_rpy = self.quaternion_to_rpy(current_spoon_pose[3:])
        current_spoon_rpy = np.concatenate((current_spoon_pose[:3], current_spoon_rpy))
        
        current_spoon_rpy[[3, 5]] = [-3.141, 0]
        
        delta = np.deg2rad(angle) - 1.571 
        current_spoon_rpy[4] = -1.571 + delta
        
        updated_spoon_quat = self.rpy_to_quaternion(
            current_spoon_rpy[3],
            current_spoon_rpy[4],
            current_spoon_rpy[5]
        )
        updated_spoon_pose = np.concatenate((current_spoon_rpy[:3], updated_spoon_quat))
        
        updated_eef_pose = self.get_eef_pose_from_spoon_pose(updated_spoon_pose, spoon_offset)
        
        final_eef_rpy = self.quaternion_to_rpy(updated_eef_pose[3:])
        final_eef_rpy = np.concatenate((updated_eef_pose[:3], final_eef_rpy))
        final_eef_rpy[[3, 5]] = [-3.141, 0]
        
        return final_eef_rpy

# ============================================================================
# MAIN
# ============================================================================

def main(args=None):
    rclpy.init(args=args)    
    ac = ArmController()

    try:
        rclpy.spin(ac)
    except KeyboardInterrupt:
        ac.get_logger().info("Closing arm_controller")
        ac.disconnect_arm()
    finally:
        ac.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
