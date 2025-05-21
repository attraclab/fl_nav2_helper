#!/bin/bash

export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/galactic/setup.bash
source ~/dev_ws/install/local_setup.bash

config_file=/home/ubuntu/dev_ws/src/fl_nav2_helper/config/nav2_params_galactic_atcrawler.yaml
map_file=/home/ubuntu/office.yaml

ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False autostart:=True map:=$map_file params_file:=$config_file &

