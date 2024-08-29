#!/usr/bin/env python3

import time
from threading import Thread

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from rclpy.duration import Duration

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import Pose

from pymoveit2 import MoveIt2


class URPlanningClient:
    def __init__(self, cartesian_planning=False):
        self.node = Node("ur_planning_client")

        ################################## Launch Parameters ######################################
        self.node.declare_parameter("synchronous", True)
        # Planner ID
        self.node.declare_parameter("planner_id", "RRTConnectkConfigDefault")
        # Declare parameters for cartesian planning
        self.node.declare_parameter("cartesian_max_step", 0.0025)
        self.node.declare_parameter("cartesian_fraction_threshold", 0.0)
        self.node.declare_parameter("cartesian_jump_threshold", 0.0)
        self.node.declare_parameter("cartesian_avoid_collisions", False)

        ################################## Miscanellous Setup ################################
        self.callback_group = ReentrantCallbackGroup()
        self.executor = MultiThreadedExecutor(2)
        self.executor.add_node(self.node)
        self.executor_thread = Thread(target=self.executor.spin, daemon=True, args=())

        ################################## MoveIt Setup 1 ######################################
        self.moveit2 = MoveIt2(
            node=self.node,
            joint_names=['shoulder_pan_joint', 
                         'shoulder_lift_joint', 
                         'elbow_joint', 
                         'wrist_1_joint', 
                         'wrist_2_joint', 
                         'wrist_3_joint'],
            base_link_name='base_link',
            end_effector_name='flange',
            group_name='arm',
            callback_group=self.callback_group,
        )

        self.executor_thread.start()
        self.node.create_rate(1.0).sleep()

        self.synch = self.node.get_parameter("synchronous").get_parameter_value().bool_value
    
        self.planner_id = self.node.get_parameter("planner_id").get_parameter_value().string_value
        self.cartesian_planning = cartesian_planning
        
        self.cartesian_max_step = self.node.get_parameter(
            "cartesian_max_step").get_parameter_value().double_value
        self.cartesian_fraction_threshold = self.node.get_parameter(
            "cartesian_fraction_threshold").get_parameter_value().double_value
        self.cartesian_jump_threshold = self.node.get_parameter(
            "cartesian_jump_threshold").get_parameter_value().double_value
        self.cartesian_avoid_collisions = self.node.get_parameter(
            "cartesian_avoid_collisions").get_parameter_value().bool_value

        self.moveit2.planner_id = self.planner_id
        self.moveit2.max_velocity = 0.5
        self.moveit2.max_acceleration = 0.5
        self.moveit2.cartesian_avoid_collisions = False
        self.moveit2.cartesian_jump_threshold = 0.0
        
        self.collision_setup(self.moveit2)

        ############################# TF Setup #########################################
        # buffer to hold the transform in a cache
        self.tf_buffer = Buffer()

        # listener. Important to spin a thread, otherwise the listen will block and no TF can be updated
        self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self.node, spin_thread=False)

    def move_to_pose(self, pose):
        self.moveit2.move_to_pose(
            pose=pose,
            cartesian=self.cartesian_planning,
        )
        if self.synch:
            self.moveit2.wait_until_executed()
    
    def get_current_pose(self) -> Pose:
        '''
        Get the current pose of the robot
        '''
        current_transform_stamp = self.tf_buffer.lookup_transform(self.moveit2.base_link_name, self.moveit2.end_effector_name, Time(), Duration(seconds=2))
        current_transform = current_transform_stamp.transform

        current_pose = Pose()
        current_pose.position.x = current_transform.translation.x
        current_pose.position.y = current_transform.translation.y
        current_pose.position.z = current_transform.translation.z
        current_pose.orientation.x = current_transform.rotation.x
        current_pose.orientation.y = current_transform.rotation.y
        current_pose.orientation.z = current_transform.rotation.z
        current_pose.orientation.w = current_transform.rotation.w

        return current_pose

    def collision_setup(self, moveit2):
        # clear all collision left from previous runs
        clear_collision_future = moveit2.clear_all_collision_objects()
        
        # Wait until the future is done
        while not clear_collision_future.done():
            time.sleep(0.1)
        
        moveit2.add_collision_box(
            id='table', position=(-1, 0.0, 0.2), quat_xyzw=(0.0, 0.0, 0.0, 1.0), size=(0.5, 1, 0.5)
        )

        moveit2.add_collision_box(
            id='left_wall', position=(0.0, -0.5, 0.5), quat_xyzw=(0.0, 0.0, 0.0, 1.0), size=(1, 0, 1)
        )

        moveit2.add_collision_box(
            id='back_wall', position=(0.5, 0, 0.5), quat_xyzw=(0.0, 0.0, 0.0, 1.0), size=(0, 1, 1)
        )

        moveit2.add_collision_box(
            id='floor', position=(0, 0, -0.1), quat_xyzw=(0.0, 0.0, 0.0, 1.0), size=(1, 1, 0)
        )