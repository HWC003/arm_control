#!/usr/bin/env python
import os
import sys
import time
from xarm.wrapper import XArmAPI

# Script to test whether the lite6 can move to a set of predefined poses without collision
# Used when setting up the robot for feeding tasks

# --- Robot Configuration ---
robot_ip = '192.168.1.190'
# Poses are defined in a structured way (e.g., using a dictionary) for clarity
robot_poses = {
    "reset": [-39.8, 182.9, 222.9, 1.899, -0.808, 3.098],

    "bite_transfer_start": [-39.8, 182.9, 222.9, 1.899, -0.808, 3.098],

    # "perception":[79.2, 128.1, 471.7, 2.691, -0.1693, 2.646],
    "perception":[77.3, 272.3, 209.3, 2.347, -0.719, 2.985],
}

def setup_arm(robot_ip, reset=False):
    """
    Connects, enables motion, and sets the operating mode/state of the xArm.
    Returns the XArmAPI object.
    """        
    print(f"Attempting to connect to xArm at {robot_ip}...")
    arm = XArmAPI(port=robot_ip, is_radian=True)
    time.sleep(0.1)
    
    # 1. Enable Motion
    ready = arm.motion_enable(enable=True)
    if ready != 0:
        max_retries = 5
        print("Motion not enabled immediately, retrying...")
        for i in range (1, max_retries+1):
            ready = arm.motion_enable(enable=True)
            if ready == 0:
                print(f"Motion enabled on attempt {i}.")
                break
            time.sleep(0.1)
        if ready != 0:
            print("ERROR: Could not enable motion. Check robot state.")
            arm.disconnect()
            return None # Return None if connection fails to be useful
            
    # 2. Set Mode and State
    # Mode 0: POSITION_CONTROL mode (typically for Cartesian movement)
    # State 0: READY state
    arm.set_mode(0) 
    arm.set_state(0)
    time.sleep(0.1)

    # 3. Optional Hard Reset
    if reset:
        print("Resetting arm...")
        arm.reset()
        time.sleep(5) # Give time for the reset to complete

    return arm # Return the arm object for further use

def main(args=None):
    # --- 1. CONNECT AND SETUP ONCE ---
    arm = setup_arm(robot_ip, reset=False)
    
    if arm is None:
        print("Exiting due to failed arm setup.")
        return

    # --- 2. EXECUTE ALL MOVEMENTS ---
    speed = 80
    mvacc = 50

    # Move to Reset Pose
    print(f"Moving to reset pose: {robot_poses['reset']}")
    code = arm.set_position(*robot_poses['reset'], speed=speed, mvacc=mvacc, wait=True)
    if code != 0:
        print(f"ERROR moving to reset pose. Code: {code}")
    
    time.sleep(2)  # Brief pause between movements

    # Move to Perception Pose
    print(f"Moving to perception pose: {robot_poses['perception']}")
    code = arm.set_position(*robot_poses['perception'], speed=speed, mvacc=mvacc, wait=True)
    if code != 0:
        print(f"ERROR moving to perception pose. Code: {code}")

    time.sleep(2)  # Brief pause between movements

    # Move to Bite Transfer Start Pose
    print(f"Moving to bite transfer start pose: {robot_poses['bite_transfer_start']}")
    code = arm.set_position(*robot_poses['bite_transfer_start'], speed=speed, mvacc=mvacc, wait=True)
    if code != 0:
        print(f"ERROR moving to bite transfer start pose. Code: {code}")

    # --- 3. DISCONNECT ONCE ---
    arm.disconnect()
    print("All movements complete. Disconnected from arm.")


if __name__ == "__main__":
    main()