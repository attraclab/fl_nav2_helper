#!/bin/bash

export ROS_DOMAIN_ID=1
source /opt/ros/galactic/setup.bash
source ~/dev_ws/install/local_setup.bash

ros2 launch fl_nav2_helper  relay_topics.launch.py

