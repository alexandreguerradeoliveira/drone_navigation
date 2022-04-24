#!/bin/bash
set -e
cd ~/catkin_ws
catkin_make

rm -f catkin_ws/src/real_time_simulator/log/log.bag

roslaunch rocket_utils template_SIL.launch
