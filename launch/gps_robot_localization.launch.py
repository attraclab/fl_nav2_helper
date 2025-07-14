from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable)
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

	fl_nav2_helper_dir = get_package_share_directory('fl_nav2_helper')
	launch_dir = os.path.join(fl_nav2_helper_dir, 'launch')

	cmd_group = GroupAction([

		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navsat_transform.launch.py'))),
			
        IncludeLaunchDescription(
			PythonLaunchDescriptionSource(os.path.join(launch_dir, 'odom_gps_imu_fusion.launch.py'))),


	])

	mavlink_sender_node =  Node(
			package='fl_nav2_helper',
			executable='mavlink_sender',
			name='mavlink_sender',
			output='screen',
			parameters=[{'convert_frame': True}]
		   )

	ld = LaunchDescription()


	ld.add_action(cmd_group)
	ld.add_action(mavlink_sender_node)

	return ld