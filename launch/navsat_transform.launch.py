from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
import yaml
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

	return LaunchDescription([
		Node(package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            arguments=['0', '0', '0','0', '0', '0', '1','base_link','gps'],
            ),

		Node(
			package='robot_localization',
			executable='navsat_transform_node',
			name='navsat_transform_node',
			output='screen',
			parameters=[os.path.join(get_package_share_directory("fl_nav2_helper"), 'config', 'navsat_transform.yaml')],
			remappings=[
				('/odometry/filtered', '/lidar_odom'),
				('/gps/fix', '/fix'),
				('/imu', '/lidar_imu')]
		   ),
])