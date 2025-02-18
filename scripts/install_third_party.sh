#!/bin/bash

# Проверка запуска из папки scripts
if [ "$(basename "$(pwd)")" != "scripts" ]; then
    echo "Error: Please execute the script from the 'scripts' folder"
    exit 1
fi

# ELP stereocamera driver
#   NB - version not set as driver is under our development
# git -C ../third_party clone https://github.com/lsd-maddrive/elp_stereo_camera

# Libviso2
# git -C ../third_party clone https://github.com/srv/viso2 -b melodic_develop_sift
# git -C ../third_party/viso2 apply ../patches/viso2.patch

# ORB-SLAM2
git -C ../third_party clone https://github.com/appliedAI-Initiative/orb_slam_2_ros -b v1.2

# rosserial
git -C ../third_party clone https://github.com/ros-drivers/rosserial.git -b noetic-devel

# Rtabmap
git -C ../third_party clone https://github.com/introlab/rtabmap.git -b 0.21.4
git -C ../third_party clone https://github.com/introlab/rtabmap_ros.git -b 0.21.4-noetic

git -C ../third_party clone https://github.com/ros-perception/image_pipeline.git -b noetic
git -C ../third_party clone https://github.com/ros-perception/vision_opencv.git -b noetic

# git -C ../third_party clone https://github.com/SteveMacenski/spatio_temporal_voxel_layer.git -b noetic-devel

# Mad Detector (package for signs detection)
git -C ../third_party clone https://github.com/lsd-maddrive/mad_detector.git

# World Creator package
git -C ../third_party clone https://github.com/PonomarevDA/world_creator.git

# Madproto - protocol for serial communication
git -C ../third_party clone https://github.com/KaiL4eK/madproto.git

# Realsense-ros
git -C ../third_party clone https://github.com/IntelRealSense/realsense-ros.git -b 2.3.2

# Ydlidar driver
git -C ../third_party clone https://github.com/YDLIDAR/ydlidar_ros_driver.git
git -C ../third_party/ydlidar_ros_driver apply ../patches/ydlidar_ros_driver.patch

# LSLidar drivers
git -C ../third_party clone https://github.com/lsd-maddrive/lslidar_ros_driver.git -b C16_V4.0 lslidar_c16_driver
git -C ../third_party clone https://github.com/lsd-maddrive/lslidar_ros_driver.git -b LS128/180/320/400S2_V1.0 lslidar_ls180s2_driver
git -C ../third_party clone https://github.com/lsd-maddrive/lslidar_ros_driver.git -b CH64W_V1.0 lslidar_ch64w_driver

# pointcloud_concatenate
git -C ../third_party clone https://github.com/aseligmann/pointcloud_concatenate.git

# GKV-3 driver
git -C ../third_party clone https://github.com/lsd-maddrive/gkv_ros_driver.git

# gazebo lidar plugin with PointCloud2 massage
git -C ../third_party clone https://bitbucket.org/DataspeedInc/velodyne_simulator.git

# include GPS plugin (for simulations)
git -C ../third_party clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo.git

# U-Blox driver
git -C ../third_party clone https://github.com/KumarRobotics/ublox -b master
# git -C ../third_party/ublox apply ../patches/ublox.patch

# rtcm_msgs for U-Blox driver
git -C ../third_party clone https://github.com/tilk/rtcm_msgs.git -b master

# nmea_msgs for U-Blox driver
git -C ../third_party clone https://github.com/ros-drivers/nmea_msgs.git -b master

# NTRIP client
git -C ../third_party clone https://github.com/LORD-MicroStrain/ntrip_client.git -b ros

# mavros_msgs for NTRIP client
git -C ../third_party clone https://github.com/mavlink/mavros.git -b master

# csv2kml convert
git -C ../third_party clone https://github.com/MapIV/kml_generator.git -b main

# hector_metapackage (mapping, imu tools)
git -C ../third_party clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git -b noetic-devel

# fork with swerve_steering_controller (4ws kinematic)
git -C ../third_party clone https://github.com/MarkNaeem/ros_controllers.git -b noetic-devel

# 4ws_steering_controller deps
git -C ../third_party clone https://github.com/ros-drivers/four_wheel_steering_msgs.git -b master
git -C ../third_party clone https://github.com/ros-controls/urdf_geometry_parser.git -b kinetic-devel
git -C ../third_party clone https://github.com/ros/geometry2.git -b noetic-devel

# gps plugin for rviz
git -C ../third_party clone https://github.com/nobleo/rviz_satellite -b master

# make tf from /odom topic
git -C ../third_party clone https://github.com/VorpalBlade/odometry_republisher.git -b master

# ZED wrapper
git -C ../third_party clone --recursive https://github.com/stereolabs/zed-ros-wrapper.git

# zed_examples and object detect rviz plugin
git -C ../third_party clone https://github.com/stereolabs/zed-ros-examples.git

# rosbag tools
git -C ../third_party clone https://github.com/srv/srv_tools.git -b melodic
git -C ../third_party clone https://github.com/neufieldrobotics/rosbag_toolkit.git

# cool rqt applications for debugging and visualizing
git -C ../third_party clone https://github.com/OTL/rqt_ez_publisher.git -b noetic-devel
git -C ../third_party clone https://github.com/shadow-robot/sr-visualization.git -b noetic-devel

# MANIPULATORS

# moveit tutorials (source https://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html)
git -C ../third_party clone https://github.com/ros-planning/moveit_tutorials.git -b master
git -C ../third_party clone https://github.com/ros-planning/panda_moveit_config.git -b noetic-devel
# moveit source
git -C ../third_party clone https://github.com/moveit/moveit_msgs.git -b master
git -C ../third_party clone https://github.com/moveit/moveit_resources.git -b master
git -C ../third_party clone https://github.com/moveit/geometric_shapes.git -b noetic-devel
git -C ../third_party clone https://github.com/moveit/srdfdom -b noetic-devel
git -C ../third_party clone https://github.com/moveit/moveit.git -b master
git -C ../third_party clone https://github.com/PickNikRobotics/rviz_visual_tools -b master
git -C ../third_party clone https://github.com/moveit/moveit_visual_tools.git -b master
git -C ../third_party clone https://github.com/moveit/moveit_tutorials.git -b master
git -C ../third_party clone https://github.com/moveit/panda_moveit_config.git -b noetic-devel

# ROS-I pkgs

# fanuc
git -C ../third_party clone https://github.com/ros-industrial/fanuc.git -b melodic-devel
git -C ../third_party clone https://github.com/ros-industrial/fanuc_experimental -b melodic-devel
#yaskawa
git -C ../third_party clone https://github.com/ros-industrial/motoman.git -b kinetic-devel
git -C ../third_party clone https://github.com/ros-industrial/motoman_experimental -b kinetic-devel
# abb
git -C ../third_party clone https://github.com/ros-industrial/abb_experimental.git -b kinetic-devel
git -C ../third_party clone https://github.com/ros-industrial/abb.git -b kinetic-devel
git -C ../third_party clone https://github.com/ros-industrial/abb_driver.git -b melodic-devel
