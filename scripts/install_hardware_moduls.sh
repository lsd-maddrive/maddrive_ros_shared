#!/bin/bash

# RealSenseSDK
if modinfo uvcvideo | grep -q "version:"; then
    if modinfo uvcvideo | grep -q "realsense"; then
        echo "The required RealSenseSDK modules are already installed"
    else
        # https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages
        sudo mkdir -p /etc/apt/keyrings
        curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | \
        sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
        echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
        sudo tee /etc/apt/sources.list.d/librealsense.list
        sudo apt-get update && sudo apt-get install -y \
            librealsense2-dkms \
            librealsense2-utils \
            librealsense2-dev \
            librealsense2-dbg
    fi
else
    echo "Could not get information about installed RealSenseSDK modules"
fi

# YDLidarSDK
# Путь к файлу ydlidar_sdkConfig.cmake
SDK_config_file="/usr/local/lib/cmake/ydlidar_sdk/ydlidar_sdkConfig.cmake"

# Проверяем, существует ли файл
if [ -f "$SDK_config_file" ]; then
    echo "Файл $SDK_config_file существует. SDK YDLidar уже установлен."
else
    git clone https://github.com/YDLIDAR/YDLidar-SDK.git
    mkdir -p YDLidar-SDK/build
    cd YDLidar-SDK/build
    cmake ..
    make
    sudo make install
fi
