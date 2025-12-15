# arm_control
Handles the robot arm control including setting up, disconnecting, restarting and moving arm to certain positions.

## Table of Contents
- [arm_control](#arm_control)
    - [Overview]()
    - [Installation](#installation)
    - [To-Dos](#todos)

## Overview
- Removes the redundancy of setting up the arm and disconnecting at the end of every function when a script calls an xarm function. This is to prevent multiple scripts from accessing the arm during the feeding cycle.
- Allows the stop and pause implementation from feeding-ui to be properly implemented. Implement with the arm_control_node rather than manager specifically.

## Installation
**Note: These instructions assume that you already have the required dependencies for the rest of the [feeding](https://github.com/janneyow/feeding) system**

1. Clone the repository in feeding 
```bash
cd ~/rfa_ws/src/feeding
git clone https://github.com/HWC003/arm_control.git
```

## ToDos:
- Get all the relevant UFactory xArm functions properly implemented in the arm_controller.py 
- Write publishers for manager.py, bite_transfer_server.py, real_scooping_env.py for the subscribers in arm_conrtoller.py