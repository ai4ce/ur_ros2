import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Joy
from std_srvs.srv import Empty

from queue import Queue


class CalibrationClient(Node):

    def __init__(self):
        super().__init__('calibration_client') # type: ignore


        ############################ Miscanellous Setup #######################################

        self._debounce_setup() # for debouncing the capture button
        self.shutter = False # when this is true, the client will issue a request to the server to capture run hand eye calibration

        ############################ Client Setup #############################################
        # Pose collecting client
        self.hand_eye_cli = self.create_client(
            srv_type=Empty, 
            srv_name='/joy_hand_eye/run_hand_eye_calibration')
        while not self.hand_eye_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('collect poses service not available, waiting again...')
        
        self.hand_eye_req = Empty.Request()

        ############################ Subscriber Setup #########################################
        # subscribe to joy topic to read joystick button press
        self.joy_sub = self.create_subscription(
            msg_type=Joy, 
            topic='/joy', 
            callback=self.joy_callback, 
            qos_profile=10)

    def joy_callback(self, msg):
        old_value = self.debounce_buffer.get() # pop the oldest read
        self.debounce_buffer.put(msg.buttons[10]) # push the newest read
        if old_value == 0 and msg.buttons[10] == 1: # button 10 is the PS button
            self.shutter = True # rising edge detected
    
    def _debounce_setup(self):
        '''
        As in any embedded system, we need to debounce the capture button.
        While we human think we press the button once, the computer actually consider the button pressed all the time during the duration of the press,
        because the polling rate is much faster than the human reaction time. 
        
        This function sets up a buffer to store the last value of the button press so that we can detect the rising edge.
        '''

        self.debounce_buffer = Queue(maxsize=1)
        self.debounce_buffer.put(0) # when nothing is pressed, the value is 0

def main():

    rclpy.init()
    executor = MultiThreadedExecutor()

    client = CalibrationClient()
    executor.add_node(client)
    

    while rclpy.ok():
        if client.shutter: # shutter down

            # send request to server to capture images
            hand_eye_future = client.hand_eye_cli.call_async(client.hand_eye_req)

            # immediately shutter up to debounce, so we don't caputre multiple images
            client.shutter = False
            
            # wait for the server to capture images
            rclpy.spin_until_future_complete(client, hand_eye_future)
            
            client.get_logger().info('Hand eye calibration done! You can now close the program.')
            
        # client.get_logger().info(f'The shutter is {image_node.shutter}')
        executor.spin_once()


    client.destroy_node()
    rclpy.shutdown()