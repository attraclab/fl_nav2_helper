from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

	ld = LaunchDescription()

	fl_nav2_dir = get_package_share_directory('fl_nav2_helper')

	map_to_odom_node = Node(package='tf2_ros',
					executable='static_transform_publisher',
					name='static_tf_pub_map_odom',
					arguments=['0.0', '0', '0.0','0', '0', '0', '1','map','odom'],
					)

	params_file = LaunchConfiguration('params_file')

	declare_params_file_cmd = DeclareLaunchArgument('params_file',
								default_value=os.path.join(fl_nav2_dir, 'config', 'single_point_nav.yaml'),
								description='Parameters file for single_point_nav_in_odom.py')


	# config_file = os.path.join(get_package_share_directory('fl_nav2_helper'),'config','params.yaml')
	# config_file = ParameterFile(params_file)
	# print(config_file)
	point_nav_node =  Node(
			package='fl_nav2_helper',
			executable='single_point_nav_in_odom',
			name='single_point_nav_in_odom_node',
			output='screen',
			parameters=[params_file]
		   )

	## will be used when run on blank map
	ld.add_action(declare_params_file_cmd)
	ld.add_action(map_to_odom_node)
	ld.add_action(point_nav_node)

	return ld