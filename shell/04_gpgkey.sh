#!/bin/sh

sudo apt update && sudo apt install curl gnupg2 lsb-release -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "check keyrings : ls /usr/share/keyrings/ ..."
ls /usr/share/keyrings/

echo "check source list : ls /etc/apt/sources.list.d/ ..."
ls /etc/apt/sources.list.d/