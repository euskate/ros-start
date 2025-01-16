#!/bin/sh

sudo apt-get update
sudo apt-get update --fix-missing
sudo apt-get install -f
sudo apt install ros-humble-desktop
