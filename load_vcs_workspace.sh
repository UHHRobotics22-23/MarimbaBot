#! /bin/bash

# Script to load a workspace from a workspace.repos file
# To be run in the ROS Industrial-CI docker image

# install vcstool
curl -s https://packagecloud.io/install/repositories/dirk-thomas/vcstool/script.deb.sh | sudo bash
sudo apt update -y
sudo apt install -y python3-vcstool

# import workspace.repos file
cd /root/target_ws/src
vcs import --skip-existing --recursive < robot_project/workspace.repos
