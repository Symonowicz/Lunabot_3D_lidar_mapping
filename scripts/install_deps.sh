#!/usr/bin/env bash
set -e

sudo apt update

sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-tf2-ros \
  ros-humble-pointcloud-to-laserscan \
  python3-rosdep

sudo rosdep init 2>/dev/null || true
rosdep update

