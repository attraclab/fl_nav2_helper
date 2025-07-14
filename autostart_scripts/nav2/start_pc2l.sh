#!/bin/bash

rosws=dev_ws
rospackage=fl_nav2_helper

#sleep 50

export DISPLAY=:0.0
export LOGFILE=/home/$USER/$rosws/src/$rospackage/autostart_scripts/pc2l.log

export ROS_DOMAIN_ID=1
source /opt/ros/galactic/setup.bash
soruce ~/dev_ws/install/local_setup.bash



while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting pc2l" >> $LOGFILE
		
		ros2 launch fl_nav2_helper pc2l.launch.py >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done
