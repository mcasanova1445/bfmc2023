#!/usr/bin/env zsh
# generated from catkin/cmake/templates/setup.zsh.in

export GAZEBO_MODEL_PATH="/Users/mc/dox/bosch/bfmc_2022/Simulator/src/models_pkg:$GAZEBO_MODEL_PATH"
export ROS_PACKAGE_PATH="/Users/mc/dox/bosch/bfmc_2022/Simulator/src:$ROS_PACKAGE_PATH"

CATKIN_SHELL=zsh

# source setup.sh from same directory as this file
_CATKIN_SETUP_DIR=$(builtin cd -q "`dirname "$0"`" > /dev/null && pwd)
emulate -R zsh -c 'source "$_CATKIN_SETUP_DIR/setup.sh"'

export ROS_MASTER_URI=http://192.168.64.7:11311
export ROS_IP=192.168.64.1
export GAZEBO_MASTER_URI=http://192.168.64.7:11345

