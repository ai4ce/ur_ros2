This package is some additions to the [official UR ROS2 driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble) that enables teleoperation (with Moveit Servo) and some more visualization. Developed at [AI4CE Lab](https://ai4ce.github.io/) at NYU.

To use these packages, please first follow the official installation for the original ROS2 driver, and then place this repo in a folder in your ROS2 workspace.

Each folder of this repo is an independent ROS2 package.

Here, I will just record some major modifications


## Teleoperation 
- Successfully implemented joint-space and twist teleoperation for fake hardware in RViz.
    - Added `JoyToServoPub_ur10e.cpp` to `ur_moveit_servo/src`
        - All 6 joints can be controlled. 
        - For twist control, pressing Home on a PS joystick can switch between the base frame and the EEF frame. 
    - I don't understand why most online discussion revolves around `forwrd_position_controller`. By default, even in fake hardware, UR driver uses `scaled_joint_trajectory_controller`, so directly publishing to the latter can make servoing work. Maybe I am missing something here.
    - In `ur_moveit_servo/config`, a servo config is provided to make sure that velocities are published to `/scaled_joint_trajectory_controller/joint_trajectory`
    - To run the teleoperation in RViz (real-world testing incoming), first run 
    ```ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=true```
    Then, in another terminal, run: 
    ```ros2 launch ur_moveit_servo teleop_sys_launch.py```
- Add speed multiplier as a launch parameter to the teleoperation system.

## EFF Integration
### Robotiq 2F-85
#### Program
- The code requires that the gripper be connected to the robot through the wrist M8 connector or the tool I/O in the control box. In other words, it does not control an independent controller but rather runs through the UR robot. 
- Another package may be written to independently control the gripper in the future if there is such a need.
- In the URDF, I turned on `use_fake_hardware` and `fake_sensor_commands`, because we don't actually control it through ROS, but through the MBUS connector. 
    - This may change if we need to write an independent driver. 
    - This is crucial for successfully configuring the gripper for the `robot_state_publisher` and smooth logging in MoveIt. Otherwise MoveIt will complain nonstop about missing the gripper, despite the fact that it can still control the robot fine.
#### Visualization
- Added `ur_robotiq_description` package to integrate the mesh and urdf of the gripper.
- Added `2f_85/collision` and `2f_85/visual` to `ur_robotiq_description/mesh`
- Added `2f_85/robotiq_2f_85_macro.urdf.xacro` to `ur_robotiq_description/urdf`
- Modified `ur_robotiq_description/urdf/ur_macro.xacro`, `ur_robotiq_description/urdf/ur.urdf.xacro` to include the macro for the gripper
- Modified `ur_robotiq_description/launch/view_ur.launch.py)` to properly load the new urdf file and visualize the robot.
