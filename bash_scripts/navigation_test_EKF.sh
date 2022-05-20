#!/bin/bash
set -e
cd ~/drone_ws

source /opt/ros/noetic/setup.bash
source /home/alex/drone_ws/devel/setup.bash
catkin_make

#rm -f drone_ws/src/real_time_simulator/log/log.bag
roslaunch drone_navigation SIL.launch
