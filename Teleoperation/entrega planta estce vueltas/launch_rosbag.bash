#!/bin/bash
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
export TURTLEBOT3_MODEL=burger
ros2 bag play rosbag\ recorrido\ llarg
