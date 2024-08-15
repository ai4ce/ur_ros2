
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
            DeclareLaunchArgument("robot_ip", default_value="192.168.0.210", description="The IP address of the robot")
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

    
    
    ur_robotiq_server_launch = Node(
        package="ur_robotiq_interface",
        executable="ur_robotiq_server",
        name="ur_robotiq_server",
    )

    ld.add_action(foxglove_launch)
    # ld.add_action(joy_launch)
    # ld.add_action(usbcam_image_server_launch)
    # ld.add_action(usbcam_image_client_launch)
    # ld.add_action(realsense_image_server_launch)
    # ld.add_action(realsense_image_client_launch)
    ld.add_action(ur_robotiq_server_launch)

    return ld