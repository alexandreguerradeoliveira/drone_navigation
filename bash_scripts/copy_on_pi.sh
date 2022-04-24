#!/bin/bash
# copy the drone_gnc package
rsync -avuzh $(rospack find drone_navigation) drone@ert.local:~/drone_ws/src
rsync -avuzh $(rospack find rocket_utils) drone@ert.local:~/drone_ws/src
rsync -avuzh $(rospack find template_gnc) drone@ert.local:~/drone_ws/src

# copy the environment loader
