#!/bin/bash

sudo apt-get update && sudo apt-get install -y \
    ros-$ROS_DISTRO-usb-cam \
    ros-$ROS_DISTRO-rosserial-server \
    ros-$ROS_DISTRO-rosserial-client \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-move-base \
    ros-$ROS_DISTRO-global-planner \
    ros-$ROS_DISTRO-teb-local-planner \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-imu-filter-madgwick \
    ros-$ROS_DISTRO-rviz-imu-plugin \
    ros-$ROS_DISTRO-hector-gazebo-plugins \
    ros-$ROS_DISTRO-gazebo-plugins \
    ros-$ROS_DISTRO-octomap-msgs \
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
    libopenvdb-dev \
    libpcap-dev \
    libgeographic-dev \
    libspnav-dev

#requirements for RS-ros
sudo apt-get purge -y \
    ros-$ROS_DISTRO-realsense2-camera \
    ros-$ROS_DISTRO-realsense2-description \
