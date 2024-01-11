#!/bin/bash

# Проверка наличия папки third_party
if [ ! -d "third_party" ]; then
    echo "Error: The folder 'third_party' was not found in the current directory. Execute the script in the 'maddrive_ros_shared' folder"
    exit 1
fi

# ELP stereocamera driver
#   NB - version not set as driver is under our development
# git -C third_party clone https://github.com/lsd-maddrive/elp_stereo_camera

# Libviso2
# git -C third_party clone https://github.com/srv/viso2 -b melodic_develop_sift
# git -C third_party/viso2 apply ../patches/viso2.patch

# ORB-SLAM2
git -C third_party clone https://github.com/appliedAI-Initiative/orb_slam_2_ros -b v1.2

# rosserial
git -C third_party clone https://github.com/ros-drivers/rosserial.git -b noetic-devel

# Rtabmap
git -C third_party clone https://github.com/introlab/rtabmap.git -b 0.21.0
git -C third_party clone https://github.com/introlab/rtabmap_ros.git -b 0.21.1-noetic

git -C third_party clone https://github.com/ros-perception/image_pipeline.git -b noetic
git -C third_party clone https://github.com/ros-perception/vision_opencv.git -b noetic

# git -C third_party clone https://github.com/SteveMacenski/spatio_temporal_voxel_layer.git -b noetic-devel

# Mad Detector (package for signs detection)
git -C third_party clone https://github.com/lsd-maddrive/mad_detector.git

# World Creator package
git -C third_party clone https://github.com/PonomarevDA/world_creator.git

# Madproto - protocol for serial communication
git -C third_party clone https://github.com/KaiL4eK/madproto.git

# hardware package reqs

# Realsense-ros (rosdep required)
git -C third_party clone https://github.com/IntelRealSense/realsense-ros.git -b 2.3.2

# Ydlidar driver
git -C third_party clone https://github.com/YDLIDAR/ydlidar_ros_driver.git
git -C third_party/ydlidar_ros_driver apply ../patches/ydlidar_ros_driver.patch
chmod 0777 third_party/ydlidar_ros_driver/startup/*
sudo sh third_party/ydlidar_ros_driver/startup/initenv.sh

# LSLidar driver
git -C third_party clone https://github.com/Lslidar/Lslidar_ROS1_driver.git

# pointCloud to PointCloud2 massage converter
git -C third_party clone https://github.com/pal-robotics-forks/point_cloud_converter.git

# GPS RTK

# include GPS plugin (for simulations)
git -C third_party clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo.git

# U-Blox driver
git -C third_party clone https://github.com/KumarRobotics/ublox -b master
# git -C third_party/ublox apply ../patches/ublox.patch

# rtcm_msgs for U-Blox driver
git -C third_party clone https://github.com/tilk/rtcm_msgs.git -b master

# nmea_msgs for U-Blox driver
git -C third_party clone https://github.com/ros-drivers/nmea_msgs.git -b master

# NTRIP client
git -C third_party clone https://github.com/LORD-MicroStrain/ntrip_client.git -b ros

# mavros_msgs for NTRIP client
git -C third_party clone https://github.com/mavlink/mavros.git -b master

# csv2kml convert
git -C third_party clone https://github.com/MapIV/kml_generator.git -b main


# hector_metapackage (mapping, imu tools)
git -C third_party clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git -b noetic-devel

# fork with swerve_steering_controller (4ws kinematic)
git -C third_party clone https://github.com/MarkNaeem/ros_controllers.git -b noetic-devel

# 4ws_steering_controller deps
git -C third_party clone https://github.com/ros-drivers/four_wheel_steering_msgs.git -b master
git -C third_party clone https://github.com/ros-controls/urdf_geometry_parser.git -b kinetic-devel
git -C third_party clone https://github.com/ros/geometry2.git -b noetic-devel

# gps plugin for rviz
git -C third_party clone https://github.com/nobleo/rviz_satellite -b master
