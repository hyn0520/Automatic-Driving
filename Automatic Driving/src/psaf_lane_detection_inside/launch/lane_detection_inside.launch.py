import os
from launch import LaunchDescription
from launch_ros.actions import Node 
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
	ld = LaunchDescription()
	config = os.path.join(
		get_package_share_directory('psaf_lane_detection_inside'),
		'config',
		'psaf_lane_detection_inside.yaml'
	)

	lane_detection_inside_node =  Node(
		package='psaf_lane_detection_inside',
		executable='lane_detection_inside',
		name='lane_detection_inside',
		parameters=[config]
	)

	ld.add_action(lane_detection_inside_node)
	return ld
