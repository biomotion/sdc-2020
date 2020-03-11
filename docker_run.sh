#!/bin/bash

BASH_OPTION=bash
if [ ! -z "$1" ]; then
    BASH_OPTION=$1
fi

if [ ! -z "$2" ]; then
    BASH_ARGS=$2
fi

echo "running $BASH_OPTION $BASH_ARGS in docker"

docker run -it --rm \
       --network=host \
       --privileged \
       --name sdc-2020 \
       -e DISPLAY=$DISPLAY \
       -e ROS_MASTER_URI=$ROS_MASTER_URI \
       -e ROS_IP=$ROS_IP \
       -v $HOME/.Xauthority:/root/.Xauthority \
       -v /dev:/dev \
       ros:kinetic-ros-core $BASH_OPTION
