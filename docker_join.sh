 
#!/bin/bash

BASH_OPTION=bash
if [ ! -z "$1" ]; then
    BASH_OPTION=$@
fi

echo "Joining docker with command: $BASH_OPTION $BASH_ARGS"

docker exec -it \
    -e ROS_MASTER_URI=$ROS_MASTER_URI \
    -e DISPLAY=${DISPLAY} \
    sdc-2020 $BASH_OPTION