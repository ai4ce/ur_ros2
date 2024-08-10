import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_srvs.srv import Empty
from sensor_msgs.msg import Image

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from scipy.spatial.transform import Rotation

import cv2
from cv_bridge import CvBridge
import yaml
import numpy as np

class CalibrationServer(Node):
    '''
    An object solely responsible for detecting the charuco board in the image.
    This is not a ROS node. It's just more convenient to have it as a class to store all the variable that got reused.
    '''
    def __init__(self):
        super().__init__('calibration_server')  # type: ignore
        
        ############################ Launch Parameters ########################################
        # parameter handling
        self.declare_parameter(name = 'imaging_system', value = 'realsense_capture')
        self.imaging_system = self.get_parameter('imaging_system').get_parameter_value().string_value
        
        ############################ Charuco Setup ###################################
        # load the yaml file
        # with open('/home/irving/Desktop/tactile_ws/src/joy_hand_eye_ROS2/joy_hand_eye/config/camera_config.yaml', 'r') as f:
        #     self.config = yaml.safe_load(f)
        self.img_size = None
        # self.k = self.config['camera_matrix']
        # self.d = self.config['distortion_coefficients']

        # construct the charuco board.
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        self.charuco_board = cv2.aruco.CharucoBoard(
            size = (8, 6), # weirdly, this is columns, rows
            squareLength=0.021,
            markerLength=0.015,
            dictionary=self.aruco_dict)
        self.charuco_board.setLegacyPattern(True) # this is needed due to OpenCV's abrupt backward compatibility breakage

        # detector parameters
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.charuco_params = cv2.aruco.CharucoParameters()
        self.charuco_detector = cv2.aruco.CharucoDetector(
            board = self.charuco_board,
            charucoParams = self.charuco_params,
            detectorParams = self.detector_params,
        )
        
        # used to calculate R and t from board to camera
        self.all_obj_points = []
        self.all_img_points = []
        
        # actual R and t from board to camera
        self.all_R_target2cam = []
        self.all_t_target2cam = []

        self.cvbridge = CvBridge() # for converting ROS images to OpenCV images

        ############################ TF Setup ########################################
        # buffer to hold the transform in a cache
        self.tf_buffer = Buffer()

        # listener. Important to spin a thread, otherwise the listen will block and no TF can be updated
        self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self, spin_thread=True)

        # actual R and t from gripper to base
        self.all_R_gripper2base = []
        self.all_t_gripper2base = []

        ############################ Service Setup ####################################
        self.my_callback_group = ReentrantCallbackGroup()

        self.hand_eye_service = self.create_service(
        srv_type=Empty, 
        srv_name='/joy_hand_eye/run_hand_eye_calibration', 
        callback=self.calibrate_callback,
        callback_group=self.my_callback_group)

        ############################ Subscriber Setup #################################
        self.image_sub = self.create_subscription(
            msg_type=Image, 
            topic=f'/{self.imaging_system}/captured_rgb_image', 
            callback=self.collect_poses_callback, 
            qos_profile=10,
            callback_group=self.my_callback_group)


    def collect_poses_callback(self, msg):
        if self.img_size is None:
            self.img_size = (msg.height, msg.width)

        image = self.cvbridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        success_flag = self.detect_charuco_board(image)

        if success_flag:
            # get the transform from the gripper to the base
            gripper2base = self.get_g2b_transform(target_frame='link_base', source_frame='link_eef')
            self.process_tf(gripper2base)

        self.get_logger().info('Current number of images: %d' % len(self.all_obj_points))
        self.get_logger().info('Current number of gripper poses: %d' % len(self.all_R_gripper2base))

    def calibrate_callback(self, request, response):
        # calibrate the hand-eye
        self.all_R_target2cam, self.all_t_target2cam = self.get_t2c_transform()

        if self.all_R_target2cam is None or self.all_t_target2cam is None:
            self.get_logger().warning('Hand-eye calibration failed. Please take more images and rerun')
            return response
        
        # select only the poses that have low reprojection error
        selected_R_gripper2base = []
        selected_t_gripper2base = []
        selected_R_target2cam = []
        selected_t_target2cam = []
        

        for i in self.selected_ids:
            selected_R_gripper2base.append(self.all_R_gripper2base[i])
            selected_t_gripper2base.append(self.all_t_gripper2base[i])
            selected_R_target2cam.append(self.all_R_target2cam[i])
            selected_t_target2cam.append(self.all_t_target2cam[i])
        
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_target2cam=selected_R_target2cam,
            t_target2cam=selected_t_target2cam,
            R_gripper2base=selected_R_gripper2base,
            t_gripper2base=selected_t_gripper2base,
        )

        self.get_logger().info('Hand-eye calibration successful')
        self.get_logger().info('R_cam2gripper: %s' % R_cam2gripper)
        self.get_logger().info('t_cam2gripper: %s' % t_cam2gripper)

        return response


    def detect_charuco_board(self, image):
        """
        Detect charuco board in image
        
        """
        charuco_corners, charuco_ids, marker_corners, marker_ids = self.charuco_detector.detectBoard(image)

        if charuco_corners is None or charuco_ids is None or marker_corners is None or marker_ids is None:
            self.get_logger().warning('No charuco board found in image. Take another image')
            return
        # render the detected board
        self.render_detected_board(image, charuco_corners, charuco_ids, marker_corners, marker_ids)
        
        obj_points, img_points = self.charuco_board.matchImagePoints(charuco_corners, charuco_ids)
        
        if len(obj_points) == 0 or len(img_points) == 0:
            self.get_logger().warning('No charuco board found in image. Take another image')
            return
        
        if len(obj_points) != len(img_points):
            self.get_logger().warning('Mismatch between object points and image points. Take another image')
            return
        
        self.all_obj_points.append(obj_points)
        self.all_img_points.append(img_points)
        
        return 1 # indicate success
       
    def render_detected_board(self, image, charuco_corners, charuco_ids, marker_corners, marker_ids):
        """
        Render the detected charuco board on the image
        """
        aruco_drawn_img = cv2.aruco.drawDetectedMarkers(image, marker_corners, marker_ids)
        charuco_drawn_img = cv2.aruco.drawDetectedCornersCharuco(aruco_drawn_img, charuco_corners, charuco_ids, cornerColor=(0, 0, 255))
        
        # show the image and block until a key is pressed
        cv2.imshow('charuco board', charuco_drawn_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    def process_tf(self, transformstamp):
        """
        Process the transform (TransformStamp) and return the translation and rotation both in numpy array
        """
        translation = np.array([transformstamp.transform.translation.x, transformstamp.transform.translation.y, transformstamp.transform.translation.z])
        quaternion = np.array([transformstamp.transform.rotation.x, transformstamp.transform.rotation.y, transformstamp.transform.rotation.z, transformstamp.transform.rotation.w])
        
        # convert quaternion to rotation matrix with scipy, which I think is more trustworthy than transforms3d
        rotation = Rotation.from_quat(quaternion)
        rotation_matrix = rotation.as_matrix()

        self.get_logger().info(f'Current Gripper Rotation: {rotation_matrix}')
        self.get_logger().info(f'Current Gripper Translation: {translation}')
        
        self.all_R_gripper2base.append(rotation)
        self.all_t_gripper2base.append(translation)

    def get_g2b_transform(self, target_frame, source_frame):
        try:
            # Timeout is important so the TF can propely initialize instead of blocking the program with None indefinitely
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, Time(), timeout=Duration(seconds=2))
            return transform
        except Exception as e:
            self.get_logger().error('Failed to get transform: %s' % e)

    def get_t2c_transform(self):
        # OpenCV's calibration function also runs PnP
        re_proj_error, camera_mtx, dist_coeffs, rvecs, tvecs, std_in, std_ex, per_view_err = cv2.calibrateCameraExtended(
            objectPoints=self.all_obj_points,
            imagePoints=self.all_img_points,
            imageSize=self.img_size, # type: ignore
            cameraMatrix=None,
            distCoeffs=None,
        )

        if re_proj_error is None:
            self.get_logger().warning('Camera calibration failed. Please take more images')
            return None, None
        if re_proj_error > 1:
            self.get_logger().warning('High overall reprojection error during camera calibration. Please take more images')
            return None, None
        
        self.get_logger().info('New camera matrix: %s' % camera_mtx)
        self.get_logger().info('New distortion coefficients: %s' % dist_coeffs)

        self.selected_ids = []

        for i in range(len(per_view_err)):
            if per_view_err[i][0] < 2:
                self.selected_ids.append(i)
        
        self.get_logger().info('Selected images and poses: %s' % self.selected_ids)

        return rvecs, tvecs



def main():

    rclpy.init()
    server = CalibrationServer()

    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
