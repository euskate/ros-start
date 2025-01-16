#!/bin/sh

echo 'alias sb="source ~/.bashrc; echo \"bashrc is reloaded\""' >> ~/.bashrc
echo "alias humble=\"source /opt/ros/humble/setup.bash; echo \'ROS Humble activated\'\"" >> ~/.bashrc