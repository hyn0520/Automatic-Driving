import os
from launch import LaunchDescription
from launch_ros.actions import Node 
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
	ld = LaunchDescription()
	config = os.path.join(
		get_package_share_directory('psaf_startbox'),
		'config',
		'psaf_startbox.yaml'
	)

	startbox_node =  Node(
		package='psaf_startbox',
		executable='startbox',
		name='startbox',
		parameters=[config]
	)

	ld.add_action(startbox_node)
	return ld
