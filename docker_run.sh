#!/bin/bash

XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<<"$xauth_list")
    if [ ! -z "$xauth_list" ]; then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# Prevent executing "docker run" when xauth failed.
if [ ! -f $XAUTH ]; then
    echo "[$XAUTH] was not properly created. Exiting..."
    exit 1
fi

BASH_OPTION=bash
if [ ! -z "$1" ]; then
    BASH_OPTION=$1
fi

if [ ! -z "$2" ]; then
    BASH_ARGS=$2
fi

echo "running $BASH_OPTION $BASH_ARGS in docker"

docker run -it --rm \
       --network host \
       --privileged \
       --name sdc-2020 \
       -e DISPLAY=$DISPLAY \
       -e ROS_MASTER_URI=$ROS_MASTER_URI \
       -e ROS_IP=$ROS_IP \
       -e QT_X11_NO_MITSHM=1 \
       -v $HOME/.Xauthority:/root/.Xauthority \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -e XAUTHORITY=$XAUTH \
       -v /dev:/dev \
       biomotion/sdc-2020:latest $BASH_OPTION
