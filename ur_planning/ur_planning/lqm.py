#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from ur_robotiq_interface.ur_robotiq_client import URRobotiqClient
from ur_planning.ur_planning_client import URPlanningClient
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Transform

def main():

    rclpy.init()
    # gripper_client = URRobotiqClient()
    planning_client = URPlanningClient()
    
    current_pose = planning_client.get_current_pose()
    print(current_pose)
    target_pose = Pose()
    target_pose.position.x = current_pose.translation.x
    target_pose.position.y = current_pose.translation.y
    target_pose.position.z = current_pose.translation.z

    target_pose.orientation.x = current_pose.rotation.x
    target_pose.orientation.y = current_pose.rotation.y
    target_pose.orientation.z = current_pose.rotation.z
    target_pose.orientation.w = current_pose.rotation.w
    target_pose.position.z += 0.3
    print(target_pose)


    planning_client.move_to(target_pose)
    
    


if __name__ == '__main__':
    main()