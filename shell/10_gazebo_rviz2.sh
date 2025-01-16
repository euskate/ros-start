#!/bin/sh

echo -e "\e[1;32m Install ROS Humble Gazebo and rviz2 \e[0m"

sudo apt install ros-humble-gazebo-* -y
sudo apt install ros-humble-robot-localization -y
sudo apt install ros-humble-rviz2 -y
sudo apt install dbus-x11 -y
sudo apt install ros-humble-gazebo-ros-pkgs -y
sudo apt install ros-humble-gazebo-ros2-control -y
sudo apt install ros-humble-cartographer -y
sudo apt install ros-humble-navigation2 -y
