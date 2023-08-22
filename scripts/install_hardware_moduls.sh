#!/usr/bin/env bash

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
        sudo apt update
        sudo apt install \
            librealsense2-dkms \
            librealsense2-utils \
            librealsense2-dev \
            librealsense2-dbg
    fi
else
    echo "Could not get information about installed RealSenseSDK modules"
fi