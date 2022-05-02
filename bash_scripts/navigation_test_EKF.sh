#!/bin/bash
set -e
cd ~/drone_ws
catkin_make

rm -f drone_ws/src/real_time_simulator/log/log.bag

roslaunch drone_navigation template_SIL.launch
#roslaunch rocket_utils template_SIL.launch
