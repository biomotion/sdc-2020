FROM osrf/ros:kinetic-desktop-full

RUN apt-get update && apt-get install -y \
    vim \
    ros-kinetic-navigation \
    && rm -rf /var/lib/apt/lists/* && \
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
