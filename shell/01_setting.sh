#!/bin/bash
echo "Ubuntu apt update & upgrade & autoremove..."
sudo apt-get update -y
sudo apt-get upgrade -y
sudo apt-get autoremove -y

sudo apt install terminator -y