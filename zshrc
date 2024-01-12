#!/bin/bash

export ZSH="$HOME/.oh-my-zsh"

source $ZSH/oh-my-zsh.sh

# ROS paths
source /opt/ros/noetic/setup.zsh
source ~/moveit_ws/devel/setup.zsh

# usefull aliases
alias rqt_tf_tree='rosrun rqt_tf_tree rqt_tf_tree'
alias rqt_robot_steering='rosrun rqt_robot_steering rqt_robot_steering'
alias rqt_reconfigure='rosrun rqt_reconfigure rqt_reconfigure'
alias rqt_plot='rosrun rqt_plot rqt_plot'
alias rqt_image_view='rosrun rqt_image_view rqt_image_view'