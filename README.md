# FAST-LIO and Nav2 helper package

This package is to help manage the pointcloud data and convert it laserscan topic by using `pointcloud_to_laserscan` package.

The `relay_topics` node is used to convert original `/Odometry` odom data not in `odom` frame, and then put it properly as `/lidar_odom` topic. 