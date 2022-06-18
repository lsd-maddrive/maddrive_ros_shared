#!/usr/bin/env bash

export CMAKE_PREFIX_PATH=/usr/local/lib/cmake/:$CMAKE_PREFIX_PATH

catkin build \
    maddrive_urdf_tools \
    ackermann_controller_plugin \
    --cmake-args -DOpenCV_DIR="/usr/local/lib/cmake/opencv4" 
    # rtabmap \
    # rtabmap_ros \
    # camera_calibration \
