#!/usr/bin/env bash

sudo apt-get install \
    ros-$ROS_DISTRO-gazebo-plugins \
    ros-$ROS_DISTRO-ddynamic-reconfigure

#requirements for RS-ros
sudo apt purge ros-$ROS_DISTRO-librealsense2
