#!/bin/bash

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/galactic/setup.bash
source ~/ros2_galactic_ws/install/local_setup.bash

#config_file=/home/ubuntu/toyota_shatai/ar_picking_cart/nav2_config/nav2_params_galactic_atbot_toyota.yaml
config_file=/home/ubuntu/toyota_shatai/ar_picking_cart/nav2_config/nav2_amcl.yaml
map_file=/home/ubuntu/office.yaml

ros2 launch nav2_bringup localization_launch.py  use_sim_time:=False autostart:=True map:=$map_file params_file:=$config_file 

