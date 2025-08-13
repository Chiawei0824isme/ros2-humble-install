#!/bin/bash
set -e

echo "==== Step 1: 設定 UTF-8 語系 ===="
locale  # check for UTF-8
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings

echo "==== Step 2: 啟用 universe 套件源 ===="
sudo apt install -y software-properties-common
sudo add-apt-repository universe

echo "==== Step 3: 安裝 curl 並加入 ROS 2 APT Source ===="
sudo apt update && sudo apt install -y curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

echo "==== Step 4: 更新套件列表並升級 ===="
sudo apt update
sudo apt upgrade -y

echo "==== Step 5: 安裝 ROS 2 Humble Desktop 與工具 ===="
sudo apt install -y ros-humble-desktop ros-humble-ros-base ros-dev-tools

echo "==== Step 6: 設定 ROS 2 環境變數 ===="
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash

echo "==== Step 7: 安裝 Gazebo ===="
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "==== Step 8: 安裝完成！測試 ROS 2 範例 ===="
echo "開新終端機後輸入以下指令進行測試："
echo "1. ros2 run demo_nodes_cpp talker      # C++ 範例發布者"
echo "2. ros2 run demo_nodes_cpp listener    # C++ 範例訂閱者"
echo "3. ros2 run demo_nodes_py talker       # Python 範例發布者"
echo "4. ros2 run demo_nodes_py listener     # Python 範例訂閱者"
echo "==== ROS 2 Humble + Gazebo 安裝完成 ===="

echo "=== Script by Chiawei ==="

