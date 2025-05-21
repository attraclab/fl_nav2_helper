#!/bin/bash

export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/galactic/setup.bash
source ~/ros2_galactic_ws/install/local_setup.bash

ros2 launch fl_nav2_helper  relay_topics.launch.py

