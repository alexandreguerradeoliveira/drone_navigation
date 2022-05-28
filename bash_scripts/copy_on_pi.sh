#!/bin/bash
# copy the drone_gnc package
DRONE_IP=192.168.71.220
rsync -avuzh ~/drone_ws/src/drone_navigation drone@"$DRONE_IP":~/drone_ws/src
rsync -avuzh ~/drone_ws/src/rocket_utils drone@"$DRONE_IP":~/drone_ws/src
#rsync -avuzh $(rospack find template_gnc) drone@"$DRONE_IP":~/drone_ws/src
rsync -avuzh ~/drone_ws/src/template_gnc drone@"$DRONE_IP":~/drone_ws/src
#rsync -avuzh $(rospack find optitrack_ekf) drone@"$DRONE_IP":~/drone_ws/src
rsync -avuzh ~/drone_ws/src/mavros_interface drone@"$DRONE_IP":~/drone_ws/src



# copy the environment loader
