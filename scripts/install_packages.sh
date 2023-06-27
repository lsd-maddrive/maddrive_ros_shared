#!/usr/bin/env bash

sudo apt-get install \
    ros-$ROS_DISTRO-gazebo-plugins \
    ros-$ROS_DISTRO-ddynamic-reconfigure \
    libpcap-dev
   
#requirements for RS-ros
sudo apt purge ros-$ROS_DISTRO-librealsense2
sudo apt-get install libpcap-dev
