#!/usr/bin/env bash

export CMAKE_PREFIX_PATH=/usr/local/lib/cmake/:$CMAKE_PREFIX_PATH

catkin build \
    realsense2_camera \
    maddrive_urdf_tools \
    ackermann_controller_plugin \
    --cmake-args -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR="/usr/local/lib/cmake/opencv4"
    # rtabmap \
    # rtabmap_ros \
    # camera_calibration \
