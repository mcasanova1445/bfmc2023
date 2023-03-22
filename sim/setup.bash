#!/usr/bin/env bash
# generated from catkin/cmake/templates/setup.bash.in

export GAZEBO_MODEL_PATH="/home/ubuntu/bfmc2023/sim/src/models_pkg:$GAZEBO_MODEL_PATH"
export ROS_PACKAGE_PATH="/home/ubuntu/bfmc2023/sim/src:$ROS_PACKAGE_PATH"

CATKIN_SHELL=bash

# source setup.sh from same directory as this file
_CATKIN_SETUP_DIR=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)
. "$_CATKIN_SETUP_DIR/setup.sh"

export ROS_MASTER_URI=http://192.168.64.7:11311
export ROS_IP=192.168.64.7
export GAZEBO_MASTER_URI=http://192.168.64.7:11345

