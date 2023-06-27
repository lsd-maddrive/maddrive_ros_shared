#!/usr/bin/env bash

THIRD_PARTY_DIR=third_party
mkdir -p $THIRD_PARTY_DIR

# GPS driver

git -C $THIRD_PARTY_DIR clone  https://github.com/KumarRobotics/ublox -b 1.4.1
git -C $THIRD_PARTY_DIR/ublox apply ../patches/ublox.patch

# ELP stereocamera driver
#   NB - version not set as driver is under our development

git -C $THIRD_PARTY_DIR clone https://github.com/lsd-maddrive/elp_stereo_camera || git -C $THIRD_PARTY_DIR pull

# Libviso2

git -C $THIRD_PARTY_DIR clone https://github.com/srv/viso2 -b melodic_develop_sift
git -C $THIRD_PARTY_DIR/viso2 apply ../patches/viso2.patch

# ORB-SLAM2

git -C $THIRD_PARTY_DIR clone https://github.com/appliedAI-Initiative/orb_slam_2_ros -b v1.2

git -C $THIRD_PARTY_DIR clone https://github.com/ros-drivers/rosserial.git -b noetic-devel

# Rtabmap

git -C $THIRD_PARTY_DIR clone https://github.com/introlab/rtabmap.git -b 0.21.0
git -C $THIRD_PARTY_DIR clone https://github.com/introlab/rtabmap_ros.git -b 0.21.1-noetic

git -C $THIRD_PARTY_DIR clone https://github.com/ros-perception/image_pipeline.git -b noetic
git -C $THIRD_PARTY_DIR clone https://github.com/ros-perception/vision_opencv.git -b noetic

git -C $THIRD_PARTY_DIR clone https://github.com/SteveMacenski/spatio_temporal_voxel_layer.git -b noetic-devel

# Realsense-ros (rosdep required)
git -C $THIRD_PARTY_DIR clone https://github.com/IntelRealSense/realsense-ros.git -b 2.3.2


# NEED TO CHOOSE ROBOT'S LIDAR

# C16 Lidar official pkg 
# git -C $THIRD_PARTY_DIR/C16 clone https://github.com/Lslidar/Lslidar_ROS1_driver.git -b C16_V2.6/2.8/3.0

# CH64W Lidar 
git -C $THIRD_PARTY_DIR clone https://github.com/Lslidar/Lslidar_ROS1_driver.git -b CH64W_V1.0
