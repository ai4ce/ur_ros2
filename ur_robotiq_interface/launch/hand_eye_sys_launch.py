
import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
)

from launch_ros.actions import Node


from ament_index_python.packages import get_package_share_directory

from launch_xml.launch_description_sources import XMLLaunchDescriptionSource



def declare_arguments():
    return LaunchDescription(
        [
            DeclareLaunchArgument("save_folder", default_value="/home/zf540/Desktop/save_folder", description="What folder to save the images to"),
            DeclareLaunchArgument("camera_namespace", default_value="xArm6", description="Namespace of the camera"),
            DeclareLaunchArgument("camera_name", default_value="D405", description="Name of the camera")
        ]
    )



def load_yaml(package_name, file_name):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_name)

    # try:
    with open(absolute_file_path) as file:
        return yaml.safe_load(file)
    # except OSError:  # parent of IOError, OSError *and* WindowsError where available
    #     return None
    
def generate_launch_description():

    save_folder_path = LaunchConfiguration("save_folder")
    camera_namespace = LaunchConfiguration("camera_namespace")
    camera_name = LaunchConfiguration("camera_name")

    ld = LaunchDescription()
    ld.add_entity(declare_arguments())
    
    foxglove_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
                os.path.join(get_package_share_directory('foxglove_bridge'), 
                                    'launch', 
                                    'foxglove_bridge_launch.xml')
        )
    )

    
    joy_launch = Node(
                executable='joy_node',
                package='joy',
                name='joy_node',
                parameters=[
                    # {'autorepeat_rate': 50.0},
                ],
            )
    usbcam_image_server_launch = Node(
        package='usbcam_capture',
        executable='usbcam_image_server',
        name='usbcam_image_server',
    )
    
    usbcam_image_client_launch = Node(
        package='usbcam_capture',
        executable='usbcam_image_client',
        name='usbcam_image_client',
        parameters=[{'save_folder': save_folder_path}],
    )

    realsense_image_server_launch = Node(
        package='realsense_capture',
        executable='realsense_image_server',
        name='realsense_image_server',
        parameters=[{'camera_namespace': camera_namespace, 
                     'camera_name': camera_name
                     }]
    )

    realsense_image_client_launch = Node(
        package='realsense_capture',
        executable='realsense_image_client',
        name='realsense_image_client',
        parameters=[{'save_folder': save_folder_path,
                     }]
    )

    calibration_client_launch = Node(
        package='joy_hand_eye',
        executable='calibration_client',
        name='calibration_client')
    
    calibration_server_launch = Node(
        package='joy_hand_eye',
        executable='calibration_server',
        name='calibration_server',
    )

    # ld.add_action(foxglove_launch)
    # ld.add_action(joy_launch)
    # ld.add_action(usbcam_image_server_launch)
    # ld.add_action(usbcam_image_client_launch)
    # ld.add_action(realsense_image_server_launch)
    # ld.add_action(realsense_image_client_launch)
    ld.add_action(calibration_client_launch)
    ld.add_action(calibration_server_launch)

    return ld