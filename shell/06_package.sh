#!/bin/sh

# Python3-pip 설치
sudo apt install -y python3-pip

# argcomplete 설치
echo 'export PATH=$PATH:$HOME/.local/bin' >> ~/.bashrc
source ~/.bashrc

pip install -U argcomplete

# colcon 빌드 도구 설치
sudo apt install python3-colcon-common-extensions -y

# rosdep 설치 및 초기화
sudo apt install python3-rosdep2 -y
rosdep update

# 시스템 업데이트
sudo apt-get update

