#!/bin/bash
# copy the drone_gnc package
DRONE_IP=192.168.64.72
rsync -avuzh $(rospack find drone_navigation) drone@"$DRONE_IP":~/drone_ws/src
rsync -avuzh $(rospack find rocket_utils) drone@"$DRONE_IP":~/drone_ws/src
#rsync -avuzh $(rospack find template_gnc) drone@"$DRONE_IP":~/drone_ws/src
rsync -avuzh ~/drone_ws/src/template_gnc drone@"$DRONE_IP":~/drone_ws/src
#rsync -avuzh $(rospack find optitrack_ekf) drone@"$DRONE_IP":~/drone_ws/src
rsync -avuzh $(rospack find mavros_interface) drone@"$DRONE_IP":~/drone_ws/src



# copy the environment loader
