#!/bin/sh

sudo apt-get update -y
sudo apt-get update --fix-missing -y
sudo apt-get install -f -y
sudo apt install ros-humble-desktop -y

ls /opt/ros/humble/