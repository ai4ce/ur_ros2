
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Empty

from threading import Thread

class URRobotiqClient:
    '''
    This class interfaces the UR Robot with a typical Robotiq gripper (2F-85/140).
    Note that it requires the gripper to be connected to the robot either through the wrist 8-pin or the tool I/O in the control box
    
    Also note that this is not a ROS2 node, but a client that can be used by other nodes to interface with the robot
    as the result, it cannot accept launch parameters. You can use another node to create this object and use it in your code
    '''
    def __init__(self):


        # super().__init__('ur_robotiq_client')  # type: ignore
        
        ############################ Launch Parameters ################################
        # parameter handling


        ############################ Service Setup ####################################
        self.my_callback_group = ReentrantCallbackGroup()
        self.node = Node('ur_robotiq_client')

        self.pick_client = self.node.create_client(
        srv_type=Empty, 
        srv_name='/ur_robotiq_interface/pick')
        while not self.pick_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('pick service not available, waiting again...')
        self.pick_request = Empty.Request()

        self.place_client = self.node.create_client(
        srv_type=Empty, 
        srv_name='/ur_robotiq_interface/place')
        while not self.pick_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('place service not available, waiting again...')
        self.place_request = Empty.Request()

        self.set_force_client = self.node.create_client(
        srv_type=Empty,
        srv_name='/ur_robotiq_interface/set_force')
        while not self.pick_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('set force service not available, waiting again...')
        self.set_force_request = Empty.Request()

        ############################ Miscanellous Setup ###############################
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.dedicated_client_thread = Thread(target=self.run)
        self.dedicated_client_thread.start()

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

    def pick(self):
        '''
        This function will send a request to the robot to pick an object
        '''
        self.pick_client.call_async(self.pick_request)
    
    def place(self):
        '''
        This function will send a request to the robot to place an object
        '''
        self.place_client.call_async(self.place_request)

def main():

    rclpy.init()
    client = URRobotiqClient()


if __name__ == '__main__':
    main()