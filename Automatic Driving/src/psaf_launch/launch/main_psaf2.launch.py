"""Start the whole pipeline with all Nodes for the old car."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

__ucbridge = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(get_package_share_directory('psaf_launch'), 'launch', 'ucbridge_old.launch.py')))

__realsense2_camera = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(get_package_share_directory('psaf_launch'), 'launch',
                 'realsense2_camera_455.launch.py')
))

__psaf_lane_detection_inside = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(get_package_share_directory('psaf_lane_detection_inside'),
                 'launch', 'lane_detection_inside.launch.py')
))

__psaf_trajectory_plan = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(get_package_share_directory('psaf_trajectory_plan'),
                 'launch', 'trajectory_plan.launch.py')
))

__psaf_controller = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(get_package_share_directory('psaf_controller'),
                 'launch', 'controller.launch.py')
))

__psaf_startbox = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(get_package_share_directory('psaf_startbox'),
                 'launch', 'startbox.launch.py')
))

__road_sign_detector = Node(
    package='road_sign_detector',  
    executable='road_sign_detector',  
    name='road_sign_detector_node', 
    output='screen',  
    parameters=[]  
)

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=['Start model car for the Carolo-Cup']),
        __ucbridge,
        __realsense2_camera,
        __psaf_lane_detection_inside,
        __psaf_trajectory_plan,
        __psaf_controller,
        __psaf_startbox,
        __road_sign_detector
   ])
