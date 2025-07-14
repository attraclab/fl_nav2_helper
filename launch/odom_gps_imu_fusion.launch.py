from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
	ld = LaunchDescription()

	### JMOAB ATCART8 ###
	base_to_imu_node = Node(package='tf2_ros',
					executable='static_transform_publisher',
					name='static_tf_pub_imu',
					arguments=['0', '0', '0','0', '0', '0', '1','base_link','imu_link'],
					)

	ekf_node =  Node(
			package='robot_localization',
			executable='ekf_node',
			name='ekf_filter_node',
			output='screen',
			parameters=[os.path.join(get_package_share_directory('fl_nav2_helper'),'config','odom_gps_imu_fusion.yaml')],
		   )

	ld.add_action(base_to_imu_node)
	ld.add_action(ekf_node)

	return ld