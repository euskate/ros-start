#!/bin/bash
echo "Ubuntu apt update & upgrade & autoremove..."
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get autoremove