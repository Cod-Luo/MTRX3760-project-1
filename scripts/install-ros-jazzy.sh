#!/usr/bin/env bash
# Run inside Ubuntu 24.04 after creating your WSL user.
# Installs the official ROS apt repository, RViz and the Gazebo integration.
set -euo pipefail
source /etc/os-release
if [[ "${ID}" != ubuntu || "${VERSION_ID}" != 24.04 ]]; then
    printf '%s\n' 'This script requires Ubuntu 24.04.' >&2
    exit 1
fi

sudo apt-get update
sudo apt-get install -y locales curl ca-certificates software-properties-common python3
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository -y universe

if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -q 'install ok installed'; then
    ros_source_version=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
        python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
    ros_source_deb=$(mktemp --suffix=.deb)
    curl -fL -o "$ros_source_deb" \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_source_version}/ros2-apt-source_${ros_source_version}.noble_all.deb"
    sudo dpkg -i "$ros_source_deb"
fi

sudo apt-get update
sudo apt-get install -y ros-jazzy-desktop ros-dev-tools ros-jazzy-ros-gz \
    ros-jazzy-robot-state-publisher ros-jazzy-turtlebot3-description \
    ros-jazzy-turtlebot3-msgs ros-jazzy-tf2
printf '%s\n' 'ROS packages installed. Follow PROJECT1.md to build and launch the maze.'
