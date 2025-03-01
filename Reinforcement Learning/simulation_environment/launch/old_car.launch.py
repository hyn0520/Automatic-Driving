import os
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():

    translator = launch_ros.actions.Node(
        package='simulation',
        name='translator',
        executable='translator_node',
        output='screen',
    )
    
    bridge = launch_ros.actions.Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/set_motor_level_msg@std_msgs/msg/Float64@ignition.msgs.Double',
         '/set_steering_level_msg@std_msgs/msg/Float64@ignition.msgs.Double', 
         '/camera@sensor_msgs/msg/Image@ignition.msgs.Image', 
         '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo', 
         '/imu_data@sensor_msgs/msg/Imu@ignition.msgs.IMU',
         '/us_front@std_msgs/msg/Float64@ignition.msgs.Double', 
         '/us_back@std_msgs/msg/Float64@ignition.msgs.Double', 
         '/us_left@std_msgs/msg/Float64@ignition.msgs.Double', 
         '/us_right@std_msgs/msg/Float64@ignition.msgs.Double', 
         '/world/car_world/control@ros_gz_interfaces/msg/WorldControl@ignition.msgs.WorldControl',
         '--ros-args', '--remap', '/camera:=/color/image_raw'],
        output='screen',
    )
    
    return LaunchDescription([
       translator,
       bridge,
       ExecuteProcess(
            cmd=['ign gazebo', '~/simulation_ws/src/simulation/simulation_projektseminar/old_model_complete.sdf'],
            output='screen',
            shell=True
        ),
       ExecuteProcess(
           cmd=['python3', '~/simulation_ws/src/simulation/gui/button.py'],
           output='screen',
           shell=True
       ),
       ExecuteProcess(
           cmd=['~/simulation_ws/src/simulation/nodes/build/./lidar_node'],
           output='screen',
           shell=True
       ),
       ExecuteProcess(
           cmd=['~/simulation_ws/src/simulation/nodes/build/./real_time_node'],
           output='screen',
           shell=True
       ),
    ])
    
#os.system("python3 ~/bridge_ws/src/simulation/start_simulation_old.py")
