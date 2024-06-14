from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

	ld = LaunchDescription()

	# map_to_odom_node = Node(package='tf2_ros',
	# 				executable='static_transform_publisher',
	# 				name='static_tf_pub_map_odom',
	# 				arguments=['0.0', '0', '0.0','0', '0', '0', '1','map','odom'],
	# 				)

	base_to_laser_node = Node(package='tf2_ros',
					executable='static_transform_publisher',
					name='static_tf_pub_imu',
					arguments=['0.2', '0', '0.0','0', '0', '0', '1','base_link','laser_frame'],
					)

	base_to_cloud_node = Node(package='tf2_ros',
					executable='static_transform_publisher',
					name='static_tf_pub_cloud',
					arguments=['0.2', '0', '0.0','0', '0', '0', '1','base_link','cloud_frame'],
					)


	relay_topics_node =  Node(
			package='fl_nav2_helper',
			executable='relay_topics',
			name='relay_topics',
			output='screen',
		   )

	## will be used when run on blank map
	#ld.add_action(map_to_odom_node)

	ld.add_action(base_to_laser_node)
	ld.add_action(relay_topics_node)
	ld.add_action(base_to_cloud_node)

	return ld