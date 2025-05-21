#!/bin/bash

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source /opt/ros/galactic/setup.bash
source ~/ros2_galactic_ws/install/local_setup.bash

ros2 launch fl_nav2_helper  all_helper.launch.py sensor_offset:=0.36 do_offset:=True pub_odom_tf:=True

