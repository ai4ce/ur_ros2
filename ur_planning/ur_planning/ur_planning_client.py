#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

from rclpy.time import Time
from rclpy.duration import Duration

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from ur_custom_msgs.srv import MoveTo, GetPose
from geometry_msgs.msg import Pose
from threading import Thread

class URPlanningClient:
    '''
    Note that this is not a ROS2 node, but a client that can be used by other nodes to interface with the robot
    as the result, it cannot accept launch parameters. You can use another node to create this object and use it in your code'''
    def __init__(self):
        # super().__init__('ur_planning_client')  # type: ignore
        
        ############################ Launch Parameters ################################
        # parameter handling


        ############################ Service Setup ####################################
        self.my_callback_group = ReentrantCallbackGroup()
        self.node = Node('ur_planning_client')

        self.moveto_client = self.node.create_client(
        srv_type=MoveTo, 
        srv_name='/ur_planning/move_to')
        while not self.moveto_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('move to service not available, waiting again...')
        self.moveto_request = MoveTo.Request()

        # self.getpose_client = self.node.create_client(
        # srv_type=GetPose,
        # srv_name='/ur_planning/get_pose')
        # while not self.getpose_client.wait_for_service(timeout_sec=1.0):
        #     self.node.get_logger().info('get pose service not available, waiting again...')

        # ############################ TF Setup #########################################
        # buffer to hold the transform in a cache
        self.tf_buffer = Buffer()

        # listener. Important to spin a thread, otherwise the listen will block and no TF can be updated
        self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self.node, spin_thread=True)

        ############################ Miscanellous Setup ###############################
        # self.executor = MultiThreadedExecutor()
        # self.dedicated_client_thread = Thread(target=self.run)
        # self.dedicated_client_thread.start()

    def run(self):
        '''
        Will spawn a dedicated thread to run the executor. This way this object can be used by other nodes
        '''
        self.executor.add_node(self.node)
        self.executor.spin()
        self.executor.remove_node(self.node)

    def __del__(self) -> None:
        self.executor.shutdown()
        self.dedicated_client_thread.join()

    def move_to(self, pose):
        '''
        Move the robot to a given pose
        '''
        self.node.get_logger().info('Moving to %r' % (pose,))
        self.moveto_request.target_pose = pose
        self.future = self.moveto_client.call_async(self.moveto_request)
        rclpy.spin_until_future_complete(self.node, self.future)
        # self.future.add_done_callback(self.move_to_callback)
    
    def move_to_callback(self, future):
        '''
        Callback for the move_to service
        '''
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))
        else:
            self.get_logger().info('Service call success %r' % (response,))

    def get_current_pose(self) -> Pose:
        '''
        Get the current pose of the robot
        '''
        # self.future = self.getpose_client.call_async(GetPose.Request())
        # self.future.add_done_callback(self.get_current_pose_callback)
        # return self.current_pose
        current_pose = self.tf_buffer.lookup_transform('base_link', 'tool0', Time(), Duration(seconds=2))
        return current_pose.transform
    
    def get_current_pose_callback(self, future):
        '''
        Callback for the get_current_pose service
        '''
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))
        else:
            self.get_logger().info('Service call success %r' % (response,))
        self.current_pose = response.pose


def main():

    rclpy.init()
    client = URPlanningClient()


if __name__ == '__main__':
    main()