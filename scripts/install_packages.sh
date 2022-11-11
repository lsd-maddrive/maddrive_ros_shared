#!/usr/bin/env bash

sudo apt-get install \
    ros-$ROS_DISTRO-gazebo-plugins \
    ros-$ROS_DISTRO-ddynamic-reconfigure
   
#requirements for RS-ros
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt purge ros-$ROS_DISTRO-librealsense2
