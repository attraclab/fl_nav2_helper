#!/bin/bash

export ROS_DOMAIN_ID=1
source ~/ros2_humble/install/local_setup.bash
source ~/rviz2_humble_ws/install/local_setup.bash
source ~/livox_ws/install/local_setup.bash
source ~/fast_lio_ws/install/local_setup.bash

ros2 launch livox_ros_driver2 msg_MID360_launch.py &


