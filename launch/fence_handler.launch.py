from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

	ld = LaunchDescription()

	show_log = LaunchConfiguration('show_log')

	declare_show_log = DeclareLaunchArgument(
		'show_log', default_value='False', description='show logging'
	)

	fence_handler_node =  Node(
			package='fl_nav2_helper',
			executable='fence_handler',
			name='fence_handler',
			output='screen',
			parameters=[{
				'show_log': show_log,}]
		   )
	ld.add_action(declare_show_log)
	ld.add_action(fence_handler_node)

	return ld