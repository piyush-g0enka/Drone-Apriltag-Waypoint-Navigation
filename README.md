# Drone Apriltag Waypoint Navigation

First install the voxl-mpa updated .deb package in the drone. This package is modified to output apriltag id in header.frame_id field of tagpose topics.

Place apriltag_localize, px4_msgs and px4_ros_com in a ros2 workspace in the drone and build the packages.
 

## Usage

''' bash

$ cd ~
$ ros2 run voxl_mpa_to_ros2 voxl_mpa_to_ros2_node 

$ cd ~
$ ros2 run apriltag_localize apriltag_localize_node 

$ cd ~
$ python3 ~/[path_to_package]/apriltag_localize/src/generate_waypoints.py

$ cd ~
$ python3 ~/[path_to_package]/px4_ros_com/src/examples/offboard_py/offboard_control.py
