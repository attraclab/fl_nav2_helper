#!/bin/bash

rosws=dev_ws
rospackage=fl_nav2_helper

#sleep 50

export DISPLAY=:0.0
export LOGFILE=/home/$USER/$rosws/src/$rospackage/autostart_scripts/fast_lio.log

export ROS_DOMAIN_ID=1
source ~/ros2_humble/install/local_setup.bash
source ~/rviz2_humble_ws/install/local_setup.bash
source ~/livox_ws/install/local_setup.bash
source ~/fast_lio_ws/install/local_setup.bash



while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting fast_lio" >> $LOGFILE
		
		ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done
