# FAST-LIO and Nav2 helper package

This package is to a helper package to work with FAST-LIO.

If want to use BEST_EFFORT QoS for all topics, please check on [fast_lio_note](./fast_lio_note/) and [livox_note](./livox_note/) directories.

`relay_topics` node is used to convert original `/Odometry` odom data not in `odom` frame, and then put it properly as `/lidar_odom` topic.

`mavlink_sender` node is to get `/Odometry` topic and put it on MAVLink message and send via UART port.