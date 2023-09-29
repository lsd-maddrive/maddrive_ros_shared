#!/usr/bin/env bash

sudo apt install \
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
    libopenvdb-dev \
    libpcap-dev \
    libspnav-dev

#requirements for RS-ros
sudo apt purge \
    ros-$ROS_DISTRO-realsense2-camera \
    ros-$ROS_DISTRO-realsense2-description \