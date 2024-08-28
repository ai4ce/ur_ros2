#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from ur_robotiq_interface.ur_robotiq_client import URRobotiqClient
from ur_planning.ur_planning_client import URPlanningClient
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Transform

import time

def main():

    rclpy.init()
    # gripper_client = URRobotiqClient()
    planning_client = URPlanningClient(cartesian_planning=True)
    
    current_pose = planning_client.get_current_pose()
    print(current_pose)
    target_pose = Pose()

    target_pose = current_pose
    target_pose.position.z += 0.1

    print(target_pose)


    planning_client.move_to_pose(target_pose)
    
    time.sleep(2)
    target_pose.position.z += 0.1
    planning_client.move_to_pose(target_pose)

    rclpy.shutdown()
    


if __name__ == '__main__':
    main()