
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_srvs.srv import Empty

import socket

PORT=63352 #PORT used by robotiq gripper. This is a fixed value

class URRobotiqServer(Node):
    '''
    This node interfaces the UR Robot with a typical Robotiq gripper (2F-85/140).
    Note that this nodes required the gripper to be connected to the robot either through the wrist 8-pin or the tool I/O.
    '''
    def __init__(self):
        super().__init__('ur_robotiq_server')  # type: ignore
        
        ############################ Launch Parameters ################################
        # parameter handling
        self.declare_parameter(name = 'robot_ip', value = '192.168.0.210')
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        
        ############################ Communication Setup ##############################
        # socket communication
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.robot_ip, PORT))
        
        ############################ Gripper Setup ####################################
        self.s.sendall(b'GET ACT\n') # get activation bit. 1 means activated, 0 means deactivated
        data = self.s.recv(2**10)
        if data != b'ACT 1\n':
            self.get_logger().info('Activating gripper')

            self.s.sendall(b'SET ACT 1\n')
            data = self.s.recv(2**10)
            if data != b'ack':
                self.get_logger().error('Failed to activate gripper')
                raise Exception('Failed to activate gripper')
            
            self.get_logger().info('Gripper activated')
        else:
            self.get_logger().info('Gripper already activated')
        
        self.force_parameter = 255 # the force parameter of the gripper. 255 is the max and the default value

        ############################ Service Setup ####################################
        self.my_callback_group = ReentrantCallbackGroup()

        self.pick_service = self.create_service(
        srv_type=Empty, 
        srv_name='/ur_robotiq_interface/pick', 
        callback=self.pick_callback,
        callback_group=self.my_callback_group)

        self.place_service = self.create_service(
        srv_type=Empty, 
        srv_name='/ur_robotiq_interface/place', 
        callback=self.place_callback,
        callback_group=self.my_callback_group)

        self.set_force_service = self.create_service(
        srv_type=Empty,
        srv_name='/ur_robotiq_interface/set_force',
        callback=self.set_force_callback,
        callback_group=self.my_callback_group)
    
    def pick_callback(self, request, response):
        '''
        This function is called when the pick service is called.
        It sends the command to the gripper to close.
        '''
        self.s.sendall(b'SET POS 255\n')
        data = self.s.recv(2**10)
        self.s.sendall(b'SET GTO 1\n')
        data = self.s.recv(2**10)
        
        return response
    
    def place_callback(self, request, response):
        '''
        This function is called when the place service is called.
        It sends the command to the gripper to open.
        '''
        self.s.sendall(b'SET POS 0\n')
        data = self.s.recv(2**10)
        self.s.sendall(b'SET GTO 1\n')
        data = self.s.recv(2**10)
        
        return response
    
    def set_force_callback(self, request, response):
        '''
        This function is called when the set_force service is called.
        It sets the force parameter of the gripper
        '''
        #TODO
        return response

def main():

    rclpy.init()
    server = URRobotiqServer()

    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()