#!/bin/bash

sudo apt-get update && sudo apt-get install -y \
    python3-catkin-tools \
    ros-$ROS_DISTRO-usb-cam \
    ros-$ROS_DISTRO-rosserial-server \
    ros-$ROS_DISTRO-rosserial-client \
    ros-$ROS_DISTRO-rosserial-python \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-move-base \
    ros-$ROS_DISTRO-global-planner \
    ros-$ROS_DISTRO-teb-local-planner \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-imu-tools \
    ros-$ROS_DISTRO-imu-filter-madgwick \
    ros-$ROS_DISTRO-rviz-imu-plugin \
    ros-$ROS_DISTRO-hector-gazebo-plugins \
    ros-$ROS_DISTRO-gazebo-plugins \
    ros-$ROS_DISTRO-octomap-msgs \
    ros-$ROS_DISTRO-octomap-rviz-plugins \
    ros-$ROS_DISTRO-ddynamic-reconfigure \
    ros-$ROS_DISTRO-libpointmatcher \
    ros-$ROS_DISTRO-spatio-temporal-voxel-layer \
    ros-$ROS_DISTRO-rqt-multiplot \
    ros-$ROS_DISTRO-libmavconn \
    ros-$ROS_DISTRO-amcl \
    ros-$ROS_DISTRO-gmapping \
    ros-$ROS_DISTRO-twist-mux \
    ros-$ROS_DISTRO-map-server \
    ros-$ROS_DISTRO-robot-upstart \
    ros-$ROS_DISTRO-foxglove-bridge \
    ros-$ROS_DISTRO-imu-filter-madgwick \
    ros-$ROS_DISTRO-laser-filters \
    ros-$ROS_DISTRO-jsk-visualization \
    ros-$ROS_DISTRO-swri-console \
    ros-$ROS_DISTRO-mapviz \
    ros-$ROS_DISTRO-mapviz-plugins \
    ros-$ROS_DISTRO-tile-map \
    ros-$ROS_DISTRO-multires-image \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-point-cloud-transport \
    ros-$ROS_DISTRO-point-cloud-transport-plugins \
    libopenvdb-dev \
    libpcap-dev \
    libgeographic-dev \
    libspnav-dev \
    rtklib \
    rtklib-qt

#requirements for RS-ros
sudo apt-get purge -y \
    ros-$ROS_DISTRO-realsense2-camera \
    ros-$ROS_DISTRO-realsense2-description

pip3 install pyserial
