#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from ur_robotiq_interface.ur_robotiq_client import URRobotiqClient
from ur_planning.ur_planning_client import URPlanningClient
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Transform

import time

def get_obstacles():
    obstacles = []
    
    table_obs = {}
    table_obs['type'] = 'box'
    table_obs['id'] = 'table'
    table_obs['position'] = (0.49, 0, -0.02)
    table_obs['quat_xyzw'] = (0, 0, 0, 1)
    table_obs['size'] = (0.97, 1.23, 0.02)
    obstacles.append(table_obs)

    right_wall_obs = {}
    right_wall_obs['type'] = 'box'
    right_wall_obs['id'] = 'right_wall'
    right_wall_obs['position'] = (0, -0.73, 0)
    right_wall_obs['quat_xyzw'] = (0, 0, 0, 1)
    right_wall_obs['size'] = (2, 0.01, 2)
    obstacles.append(right_wall_obs)

    back_wall_obs = {}
    back_wall_obs['type'] = 'box'
    back_wall_obs['id'] = 'back_wall'
    back_wall_obs['position'] = (-0.6, 0, 0)
    back_wall_obs['quat_xyzw'] = (0, 0, 0, 1)
    back_wall_obs['size'] = (0.01, 2, 2)
    obstacles.append(back_wall_obs)

    return obstacles

def put_first_on_second(arg1, arg2):
    '''
    LMP definition for putting the first object on the second object
    '''
    pass

def get_obj_pos(self, obj_name):
    '''
    LMP definition for getting the position of an object given its name
    '''
    pass

def main():

    rclpy.init()

        
    # gripper_client = URRobotiqClient()
    planning_client = URPlanningClient(cartesian_planning=False)
    

    obstacles = get_obstacles()
    planning_client.collision_setup(obstacles=obstacles)


    


if __name__ == '__main__':
    main()