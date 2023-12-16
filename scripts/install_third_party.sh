#!/bin/bash

THIRD_PARTY_DIR=third_party
mkdir -p $THIRD_PARTY_DIR

# ELP stereocamera driver
#   NB - version not set as driver is under our development
# git -C $THIRD_PARTY_DIR clone https://github.com/lsd-maddrive/elp_stereo_camera

# Libviso2
# git -C $THIRD_PARTY_DIR clone https://github.com/srv/viso2 -b melodic_develop_sift
# git -C $THIRD_PARTY_DIR/viso2 apply ../patches/viso2.patch

# ORB-SLAM2
git -C $THIRD_PARTY_DIR clone https://github.com/appliedAI-Initiative/orb_slam_2_ros -b v1.2

# rosserial
git -C $THIRD_PARTY_DIR clone https://github.com/ros-drivers/rosserial.git -b noetic-devel

# Rtabmap
git -C $THIRD_PARTY_DIR clone https://github.com/introlab/rtabmap.git -b 0.21.0
git -C $THIRD_PARTY_DIR clone https://github.com/introlab/rtabmap_ros.git -b 0.21.1-noetic

git -C $THIRD_PARTY_DIR clone https://github.com/ros-perception/image_pipeline.git -b noetic
git -C $THIRD_PARTY_DIR clone https://github.com/ros-perception/vision_opencv.git -b noetic

# git -C $THIRD_PARTY_DIR clone https://github.com/SteveMacenski/spatio_temporal_voxel_layer.git -b noetic-devel

# Mad Detector (package for signs detection)
git -C $THIRD_PARTY_DIR clone https://github.com/lsd-maddrive/mad_detector.git

# World Creator package
git -C $THIRD_PARTY_DIR clone https://github.com/PonomarevDA/world_creator.git

# Madproto - protocol for serial communication
git -C $THIRD_PARTY_DIR clone https://github.com/KaiL4eK/madproto.git

# hardware package reqs

# Realsense-ros (rosdep required)
git -C $THIRD_PARTY_DIR clone https://github.com/IntelRealSense/realsense-ros.git -b 2.3.2

# Ydlidar driver
git -C $THIRD_PARTY_DIR clone https://github.com/YDLIDAR/ydlidar_ros_driver.git
git -C $THIRD_PARTY_DIR/ydlidar_ros_driver apply ../patches/ydlidar_ros_driver.patch
chmod 0777 $THIRD_PARTY_DIR/ydlidar_ros_driver/startup/*
sudo sh $THIRD_PARTY_DIR/ydlidar_ros_driver/startup/initenv.sh

# LSLidar driver
git -C $THIRD_PARTY_DIR clone https://github.com/Lslidar/Lslidar_ROS1_driver.git

# pointCloud to PointCloud2 massage converter
git -C $THIRD_PARTY_DIR clone https://github.com/pal-robotics-forks/point_cloud_converter.git

# GPS RTK

# include GPS plugin (for simulations)
git -C $THIRD_PARTY_DIR clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo.git

# U-Blox driver
git -C $THIRD_PARTY_DIR clone https://github.com/KumarRobotics/ublox -b master
# git -C $THIRD_PARTY_DIR/ublox apply ../patches/ublox.patch

# rtcm_msgs for U-Blox driver
git -C $THIRD_PARTY_DIR clone https://github.com/tilk/rtcm_msgs.git -b master

# nmea_msgs for U-Blox driver
git -C $THIRD_PARTY_DIR clone https://github.com/ros-drivers/nmea_msgs.git -b master

# NTRIP client
git -C $THIRD_PARTY_DIR clone https://github.com/LORD-MicroStrain/ntrip_client.git -b ros

# mavros_msgs for NTRIP client
git -C $THIRD_PARTY_DIR clone https://github.com/mavlink/mavros.git -b master

# csv2kml convert
git -C $THIRD_PARTY_DIR clone https://github.com/MapIV/kml_generator.git -b main


# hector_metapackage (mapping, imu tools)
git -C $THIRD_PARTY_DIR clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git -b noetic-devel

# fork with swerve_steering_controller (4ws kinematic)
git -C $THIRD_PARTY_DIR clone https://github.com/MarkNaeem/ros_controllers.git -b noetic-devel

# 4ws_steering_controller deps
git -C $THIRD_PARTY_DIR clone https://github.com/ros-drivers/four_wheel_steering_msgs.git -b master
git -C $THIRD_PARTY_DIR clone https://github.com/ros-controls/urdf_geometry_parser.git -b kinetic-devel
git -C $THIRD_PARTY_DIR clone https://github.com/ros/geometry2.git -b noetic-devel

# gps plugin for rviz
git -C $THIRD_PARTY_DIR clone https://github.com/nobleo/rviz_satellite -b master
