#!/usr/bin/env python3

import time
from threading import Thread
import numpy as np
import math
from typing import Any, List, Optional, Tuple, Union

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from rclpy.duration import Duration

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory


from pymoveit2 import MoveIt2


class URPlanningClient:
    def __init__(self, cartesian_planning=False, pipeline_id = '', planner_id="RRTConnectkConfigDefault"):
        '''
        cartesian_planning: If True, use MoveIt's cartesian planning. If False, use joint space planning
        pipeline_id: ID of the pipeline to use for planning. ['ompl', 'pilz_industrial_motion_planner']. If empty, use the default pipeline
        planner_id: The exact planner to use. Changes based on pipeline_id. If empty, use the default planner
        '''
        self.node = Node("xarm_planning_client")

        ################################## Launch Parameters ######################################
        self.node.declare_parameter("synchronous", True)
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
        
        ################################## MoveIt Setup ######################################
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
    
        self.cartesian_planning = cartesian_planning
        
        self.cartesian_max_step = self.node.get_parameter(
            "cartesian_max_step").get_parameter_value().double_value
        self.cartesian_fraction_threshold = self.node.get_parameter(
            "cartesian_fraction_threshold").get_parameter_value().double_value
        self.cartesian_jump_threshold = self.node.get_parameter(
            "cartesian_jump_threshold").get_parameter_value().double_value
        self.cartesian_avoid_collisions = self.node.get_parameter(
            "cartesian_avoid_collisions").get_parameter_value().bool_value

        self.moveit2.pipeline_id = pipeline_id
        self.moveit2.planner_id = planner_id

        self.moveit2.max_velocity = 0.5
        self.moveit2.max_acceleration = 0.5
        self.moveit2.cartesian_avoid_collisions = False
        self.moveit2.cartesian_jump_threshold = 0.0
        
        ############################# TF Setup #########################################
        # buffer to hold the transform in a cache
        self.tf_buffer = Buffer()

        # listener. Important to spin a thread, otherwise the listen will block and no TF can be updated
        self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self.node, spin_thread=False)

        ############################# Publisher Setup ##################################

        
        ############################# Subscriber Setup ##################################


    def move_to_pose(self, pose):
        self.moveit2.move_to_pose(
            pose=pose,
            cartesian=self.cartesian_planning,
        )
        if self.synch:
            self.moveit2.wait_until_executed()
    
    def plan_to_pose(self, pose) -> Optional[JointTrajectory]:
        if self.moveit2.pipeline_id == 'pilz_industrial_motion_planner':
            frame_id = 'world' # a bug in pilz_industrial_motion_planner requires a frame_id to be set
        else:
            frame_id = None
        planned_traj = self.moveit2.plan(pose=pose, cartesian=self.cartesian_planning, frame_id=frame_id)
        
        return planned_traj
    
    def execute_plan(self, plan: JointTrajectory):
        self.moveit2.execute(plan)
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

    def collision_setup(self, obstacles):
        # clear all collision left from previous runs
        clear_collision_future = self.moveit2.clear_all_collision_objects()
        
        while not clear_collision_future.done(): # type: ignore
            time.sleep(0.1)
        
        for obstacle in obstacles:
            if obstacle['type'] == 'sphere':
                self.moveit2.add_collision_sphere(
                    id=obstacle['id'], position=obstacle['position'], radius=obstacle['radius']
                )
            if obstacle['type'] == 'box':
                self.moveit2.add_collision_box(
                    id=obstacle['id'], position=obstacle['position'], quat_xyzw=obstacle['quat_xyzw'], size=obstacle['size']
                )
        # moveit2.add_collision_box(
        #     id='table', position=(-1, 0.0, 0.2), quat_xyzw=(0.0, 0.0, 0.0, 1.0), size=(0.5, 1, 0.5)
        # )

        # moveit2.add_collision_box(
        #     id='left_wall', position=(0.0, -0.5, 0.5), quat_xyzw=(0.0, 0.0, 0.0, 1.0), size=(1, 0, 1)
        # )

        # moveit2.add_collision_box(
        #     id='back_wall', position=(0.5, 0, 0.5), quat_xyzw=(0.0, 0.0, 0.0, 1.0), size=(0, 1, 1)
        # )

        # moveit2.add_collision_box(
        #     id='floor', position=(0, 0, -0.1), quat_xyzw=(0.0, 0.0, 0.0, 1.0), size=(1, 1, 0)
        # )

    def surround_and_lock(self, center, num_waypoints):
        '''
        Generate a list of poses around a circular path and lock the end-effector to the object
        center: 3D position of the center of the circle (the object)
        height: Constant height for the end-effector
        num_poses: Number of poses to generate, including the first pose
        '''

        # Generate waypoints on the circular path
        current_pose = self.get_current_pose()
        self._circular_path_initial_position = np.array([current_pose.position.x, 
                                                         current_pose.position.y, 
                                                         current_pose.position.z])
        
        self._circular_path_initial_orientation = np.array([current_pose.orientation.x, 
                                                            current_pose.orientation.y, 
                                                            current_pose.orientation.z, 
                                                            current_pose.orientation.w])
        self._compute_initial_angle(center) # Compute the initial angle between the current position and the object's center
        # self.moveit2.set_path_orientation_constraint(
        #     quat_xyzw=current_pose.orientation,
        #     tolerance= (3.14159, 1.0, 3.14159),
        #     parameterization=0,
        # )
        waypoints = []
        for i in range(1, num_waypoints):
            angle = 2 * math.pi * i / num_waypoints
            pose = self._compute_pose_on_circle(center, angle)
            waypoints.append(pose)

        return waypoints
    
    def _start_state_callback(self, msg):
        self.joint_state = msg

    def _compute_pose_on_circle(self, center, angle):
        """
        Compute a pose on a circular path at a given angle.
        center: 3D position of the center of the circle (the object)
        angle: Angle around the circle (radians)
        height: Constant height for the end-effector
        """
        total_angle = self._circular_path_angle_offset + angle
        radius = np.linalg.norm(self._circular_path_initial_position - center)
        
        # Calculate the position on the circle
        x = center[0] + radius * math.cos(total_angle)
        y = center[1] + radius * math.sin(total_angle)
        z = self._circular_path_initial_position[2]

        # Camera's current position
        camera_position = np.array([x, y, z])

        # Compute the orientation that points toward the center
        orientation = self._gaze_at(center, camera_position)

        # Create a Pose object
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]

        return pose

    def _gaze_at(self, center_point, ideal_camera_position, initial_y=np.array([0, 1, 0])):
        """
        Calculate the quaternion that rotates the camera to look at the center point.
        center_point: 3D position of the object in world frame
        ideal_camera_position: 3D position of the camera (end-effector) in world frame
        initial_up: Initial up direction of the EEF (default is along the y-axis)
        """
        # Vector from the camera (end-effector) to the object
        direction = center_point - ideal_camera_position
        direction = self._normalize(direction)

        # Define a reference vector (camera's forward direction in world coordinates)
        # Assuming the camera is initially pointing along the z-axis of the end-effector
        ref_forward = np.array([0, 0, 1])

        # Calculate the rotation matrix using two vectors (from reference to target direction)
        rotation, rssd = Rotation.align_vectors([direction], [ref_forward], return_sensitivity=False) # type: ignore

        # Convert rotation matrix to quaternion
        quaternion = rotation.as_quat()
        # quaternion = self._align_y_axes_quaternions(quaternion)

        # current_y = rotation.apply(initial_y)

        # # Desired up direction is along the world's z-axis or initial up (e.g., [0, 0, 1])
        # world_up = np.array([0, 0, 1])
        
        # correction_axis = np.cross(current_y, initial_y)
        # if np.linalg.norm(correction_axis) > 1e-6:  # Prevent division by zero
        #     correction_axis = self._normalize(correction_axis)
        #     correction_angle = math.acos(np.dot(current_y, initial_y))
        #     correction_rotation = Rotation.from_rotvec(correction_angle * correction_axis)
        #     # Apply correction to the rotation
        #     rotation = correction_rotation * rotation

        return quaternion
    
    def _compute_initial_angle(self, center):
        """
        Calculate the initial angle between the current position and the object's center.
        center: 3D position of the object
        current_position: Current position of the end-effector
        """
        delta_x = self._circular_path_initial_position[0] - center[0]
        delta_y = self._circular_path_initial_position[1] - center[1]
        
        # Calculate the angle in the XY plane
        self._circular_path_angle_offset = math.atan2(delta_y, delta_x)
        
    def _normalize(self, v):
        """ Normalize a vector """
        return v / np.linalg.norm(v)
        
    def _align_y_axes_quaternions(self, q1):
        """
        Rotates quaternion q1 around the z-axis so that its y-axis is most aligned
        with the y-axis of quaternion q2 in the x-y plane.
        """

        # Convert quaternions to rotation matrices
        R1 = Rotation.from_quat(q1).as_matrix()
        R2 = Rotation.from_quat(self._circular_path_initial_orientation).as_matrix()

        # Extract the y-axes from the rotation matrices
        y_axis_1 = R1[:2, 1]  # x-y components of the y-axis of q1
        y_axis_2 = R2[:2, 1]  # x-y components of the y-axis of q2

        # Normalize the y-axes
        y_axis_1 /= np.linalg.norm(y_axis_1)
        y_axis_2 /= np.linalg.norm(y_axis_2)

        # Compute the angle between the two Y-axes using the dot product
        dot_product = np.dot(y_axis_1, y_axis_2)
        angle = np.arccos(np.clip(dot_product, -1.0, 1.0))  # Ensure the value is within valid range

        # Determine the sign of the angle using the cross product
        cross_product = np.cross(y_axis_1, y_axis_2)
        if cross_product < 0:
            angle = -angle  # Adjust the sign of the angle if needed

        # Create a quaternion for the yaw rotation around the z-axis
        yaw_rotation = Rotation.from_euler('z', angle).as_quat()

        # Apply the yaw rotation to the first quaternion
        new_q1 = (Rotation.from_quat(q1)*Rotation.from_quat(yaw_rotation)).as_quat()

        return new_q1