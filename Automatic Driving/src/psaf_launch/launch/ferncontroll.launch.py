"""Start the whole pipeline with all Nodes for the old car."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

__ucbridge = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(get_package_share_directory('psaf_launch'), 'launch', 'ucbridge_old.launch.py')))

__psaf_firststeps = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(get_package_share_directory('psaf_firststeps'),
                 'launch', 'firststeps.launch.py')
))

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=['Start model car for the Carolo-Cup']),
        __ucbridge,

        __psaf_firststeps

   ])
