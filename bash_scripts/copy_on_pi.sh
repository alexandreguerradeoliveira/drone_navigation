#!/bin/bash
# copy the drone_gnc package
rsync -avuzh $(rospack find drone_navigation) drone@ert.local:~/drone_ws/src
rsync -avuzh $(rospack find real_time_simulator) drone@ert.local:~/drone_ws/src
# copy the environment loader
