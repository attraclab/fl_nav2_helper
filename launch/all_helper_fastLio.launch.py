from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

	fl_nav2_helper_dir = get_package_share_directory('fl_nav2_helper')
	launch_dir = os.path.join(fl_nav2_helper_dir, 'launch')

	sensor_offset = LaunchConfiguration('sensor_offset')
	do_offset = LaunchConfiguration('do_offset')
	pub_odom_tf = LaunchConfiguration('pub_odom_tf')

	declare_sensor_offset = DeclareLaunchArgument(
		'sensor_offset', default_value='0.2', description='The sensor offset from base_link to lidar position'
	)

	declare_do_offset = DeclareLaunchArgument(
		'do_offset', default_value='True', description='Using offset or not'
	)

	declare_pub_odom_tf = DeclareLaunchArgument(
		'pub_odom_tf', default_value='False', description='Publish odom base_link TF'
	)


	cmd_group = GroupAction([

		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(os.path.join(launch_dir, 'pc2l.launch.py'))),

		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(os.path.join(launch_dir,'relay_topics.launch.py')),
			launch_arguments={'sensor_offset': sensor_offset,
							  'do_offset': do_offset,
							  'pub_odom_tf': pub_odom_tf}.items()),

	])

	ld = LaunchDescription()

	ld.add_action(declare_sensor_offset)
	ld.add_action(declare_do_offset)
	ld.add_action(declare_pub_odom_tf)
	ld.add_action(cmd_group)

	return ld