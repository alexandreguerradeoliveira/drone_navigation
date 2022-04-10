#!/bin/bash
# copy the drone_gnc package
rsync -avuzh $(rospack find rocket_gnc) drone@ert.local:~/drone_ws/src
rsync -avuzh $(rospack find real_time_simulator) drone@ert.local:~/drone_ws/src
# copy the environment loader
