from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

	ld = LaunchDescription()

	# map_to_odom_node = Node(package='tf2_ros',
	# 				executable='static_transform_publisher',
	# 				name='static_tf_pub_map_odom',
	# 				arguments=['0.0', '0', '0.0','0', '0', '0', '1','map','odom'],
	# 				)

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

	base_to_laser_node = Node(package='tf2_ros',
					executable='static_transform_publisher',
					name='static_tf_pub_imu',
					arguments=[sensor_offset, '0', '0','0', '0', '0', '1','base_link','laser_frame'],
					)

	base_to_cloud_node = Node(package='tf2_ros',
					executable='static_transform_publisher',
					name='static_tf_pub_cloud',
					arguments=[sensor_offset, '0', '0','0', '0', '0', '1','base_link','cloud_frame'],
					)


	relay_topics_node =  Node(
			package='fl_nav2_helper',
			executable='relay_topics',
			name='relay_topics',
			output='screen',
			parameters=[{'sensor_offset': sensor_offset,
						'do_offset': do_offset,
						'pub_odom_tf': pub_odom_tf}]
		   )

	## will be used when run on blank map
	#ld.add_action(map_to_odom_node)
	ld.add_action(declare_sensor_offset)
	ld.add_action(declare_do_offset)
	ld.add_action(declare_pub_odom_tf)
	ld.add_action(base_to_laser_node)
	ld.add_action(relay_topics_node)
	ld.add_action(base_to_cloud_node)

	return ld