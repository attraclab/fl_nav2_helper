#!/bin/bash

export ROS_DOMAIN_ID=1
source /opt/ros/galactic/setup.bash
source ~/dev_ws/install/local_setup.bash

config_file=/home/ubuntu/dev_ws/src/fl_nav2_helper/config/mapper_params_online_async.yaml

ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$config_file &

