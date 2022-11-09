#!/usr/bin/env bash

export CMAKE_PREFIX_PATH=/usr/local/lib/cmake/:$CMAKE_PREFIX_PATH

catkin build \
    realsense2_camera \
    --cmake-args -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR="/usr/local/lib/cmake/opencv4"
    maddrive_urdf_tools \
    ackermann_controller_plugin \
    # rtabmap \
    # rtabmap_ros \
    # camera_calibration \
