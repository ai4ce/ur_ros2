import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from ur_robotiq_interface.ur_robotiq_client import URRobotiqClient

def main():

    rclpy.init()
    server = URRobotiqClient()

    server.place()

    


if __name__ == '__main__':
    main()