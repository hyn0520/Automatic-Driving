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


__imitation_learning = Node(
    package='imitation_learning',  
    executable='imitation_learning',  
    name='steering_node',  
    output='screen',  
    parameters=[]  
)

__psaf_startbox = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(get_package_share_directory('psaf_startbox'),
                 'launch', 'startbox.launch.py')
))


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=['Start model car for the Carolo-Cup']),
        __ucbridge,
        __realsense2_camera,
        __imitation_learning,
        __psaf_startbox
   ])
