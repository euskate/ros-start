#!/bin/sh

source /opt/ros/humble/setup.bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/
colcon build --symlink-install
ls