#!/bin/bash

rosws=dev_ws
rospackage=fl_nav2_helper

sleep 20

export DISPLAY=:0.0
export LOGFILE=/home/$USER/$rosws/src/$rospackage/autostart_scripts/all_helper.log
export ROS_DOMAIN_ID=0

source ~/ros2_humble/install/local_setup.bash
source ~/livox_ws/install/local_setup.bash
source ~/fast_lio_ws/install/local_setup.bash
source ~/ros2_vision_ws/install/local_setup.bash
source ~/dev_ws/install/local_setup.bash
source ~/nav2_ws/install/local_setup.bash
source ~/micro_ros_ws/install/local_setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export OPENBLAS_CORETYPE=ARMV8

while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting fast_lio" >> $LOGFILE
		
		ros2 launch fl_nav2_helper  all_helper_fastLio.launch.py sensor_offset:=0.0 do_offset:=False pub_odom_tf:=True >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done
