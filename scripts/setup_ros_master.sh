#!/bin/bash

export ROS_HOSTNAME=$(hostname)
export ROS_MASTER_URI=http://8.8.8.8:11311

echo "Master: $ROS_MASTER_URI / Hostname: $ROS_HOSTNAME"
