from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

	ld = LaunchDescription()

	convert_frame = LaunchConfiguration('convert_frame')
	fix_wait_sec = LaunchConfiguration('fix_wait_sec')
	wait_gps = LaunchConfiguration('wait_gps')
	send_obstacle = LaunchConfiguration('send_obstacle')

	declare_convert_frame = DeclareLaunchArgument(
		'convert_frame', default_value='True', description='convert ENU to NED frame'
	)
	declare_fix_wait_sec = DeclareLaunchArgument(
		'fix_wait_sec', default_value='120.0', description='How much to wait after got rtk fixed'
	)
	declare_wait_gps = DeclareLaunchArgument(
		'wait_gps', default_value='True', description='wait for gps'
	)
	declare_send_obstacle = DeclareLaunchArgument(
		'send_obstacle', default_value='False', description='send obstacle distance'
	)

	mavlink_sender_node =  Node(
			package='fl_nav2_helper',
			executable='mavlink_sender',
			name='mavlink_sender',
			output='screen',
			parameters=[{
				'convert_frame': convert_frame,
				'fix_wait_sec': fix_wait_sec,
				'wait_gps': wait_gps,
				'send_obstacle': send_obstacle}]
		   )
	ld.add_action(declare_convert_frame)
	ld.add_action(declare_fix_wait_sec)
	ld.add_action(declare_wait_gps)
	ld.add_action(declare_send_obstacle)
	ld.add_action(mavlink_sender_node)

	return ld