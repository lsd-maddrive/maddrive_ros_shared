#!/bin/bash

if [ $(id -u) -ne 0 ]
then
    echo "This script must be run as root";
    exit 1;
fi

if modinfo uvcvideo | grep -q "version:"; then
    if modinfo uvcvideo | grep -q "realsense"; then
        echo "The required RealSenseSDK modules are already installed"
    else
        # based on https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages
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
SDK_config_file="/usr/local/lib/cmake/ydlidar_sdk/ydlidar_sdkConfig.cmake"

if [ -f "$SDK_config_file" ]; then
    echo "The YDLidar SDK is already installed"
else
    # based on https://www.ydlidar.com/Public/upload/files/2022-06-21/YDLIDAR%20X2%20Lidar%20User%20Manual%20V1.3(211228).pdf
    git clone https://github.com/YDLIDAR/YDLidar-SDK.git
    mkdir -p YDLidar-SDK/build
    cd YDLidar-SDK/build
    # based on https://raw.githubusercontent.com/YDLIDAR/ydlidar_ros_driver/master/startup/initenv.sh
    echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar%n"' > /etc/udev/rules.d/ydlidar.rules
    echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar%n"' > /etc/udev/rules.d/ydlidar-V2.rules
    echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar%n"' > /etc/udev/rules.d/ydlidar-2303.rules

    cmake ..
    make
    sudo make install
    cd ../..
    rm -rf YDLidar-SDK
fi

# YAHBOOM IMU device
rules_file="/etc/udev/rules.d/yahboom_imu.rules"

if [ -f "$rules_file" ]; then
    echo "The YAHBOOM IMU device is already installed"
else
    # based on https://github.com/YahboomTechnology/10-axis_IMU_Module
    echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="tty_Yahboom%n", GROUP="dialout"' > /etc/udev/rules.d/yahboom_imu.rules

    service udev reload
    sleep 2
    service udev restart
fi

# simpleRTK2B device
rules_file="/etc/udev/rules.d/simpleRTK2B.rules"

if [ -f "$rules_file" ]; then
    echo "The simpleRTK2B device is already installed"
else
    echo 'KERNEL=="ttyACM[0-9]*", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", MODE:="0777", SYMLINK+="tty_Ardusimple%n", GROUP="dialout"' > /etc/udev/rules.d/simpleRTK2B.rules

    service udev reload
    sleep 2
    service udev restart
    sudo udevadm trigger
fi

# U-blox c94-m8p device
rules_file="/etc/udev/rules.d/ublox_c94-m8p.rules"

if [ -f "$rules_file" ]; then
    echo "The U-blox C94-M8P device is already installed"
else
    echo 'KERNEL=="ttyACM[0-9]*", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a8", MODE:="0777", SYMLINK+="tty_Ublox_C94-M8P%n", GROUP="dialout"' > /etc/udev/rules.d/ublox_c94-m8p.rules

    service udev reload
    sleep 2
    service udev restart
    sudo udevadm trigger
fi
