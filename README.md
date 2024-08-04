This package is some additions to the [official UR ROS2 driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble) that enables teleoperation (with Moveit Servo) and some more visualization. Developed at [AI4CE Lab](https://ai4ce.github.io/) at NYU.

Here, I will just record some major modifications


## Teleoperation 
- Successfully implemented joint-space and twist teleoperation for fake hardware in RViz.
    - Added `JoyToServoPub_ur10e.cpp` to `ur_moveit_servo/src`
        - All 6 joints can be controlled. 
        - For twist control, pressing Home on a PS joystick can switch between the base frame and the EEF frame. (TODO: Properly debounce the button)
        - Add speed multiplier as a launch parameter to the teleoperation system.
    - I don't understand why most online discussion revolves around `forwrd_position_controller`. By default, even in fake hardware, UR driver uses `scaled_joint_trajectory_controller`, so directly publishing to the latter can make servoing work. Maybe I am missing something here.
    - In `ur_moveit_servo/config`, a servo config is provided to make sure that velocities are published to `/scaled_joint_trajectory_controller/joint_trajectory`
    - To run the teleoperation in RViz (real-world testing incoming), first run 
    ```ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=true```
    Then, in another terminal, run: 
    ```ros2 launch ur_moveit_servo teleop_sys_launch.py```