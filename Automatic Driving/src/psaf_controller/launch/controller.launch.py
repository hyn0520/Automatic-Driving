import os
from launch import LaunchDescription
from launch_ros.actions import Node 
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
	ld = LaunchDescription()
	config = os.path.join(
		get_package_share_directory('psaf_controller'),
		'config',
		'psaf_controller.yaml'
	)

	controller_node =  Node(
		package='psaf_controller',
		executable='controller',
		name='controller',
		parameters=[config]
	)

	ld.add_action(controller_node)
	return ld
