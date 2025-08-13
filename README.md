# ROS 2 Humble Installation & Verification Guide
![Build Status](https://img.shields.io/badge/build-passing-brightgreen)
![License](https://img.shields.io/badge/license-MIT-blue)
![ROS 2 Version](https://img.shields.io/badge/ROS2-Humble-orange)

# Table of Contents
1. [Install](#1install)
2. [Build Workspace](#2build)
3. [Check](#3check)
   - [Check ROS 2 Related](#check-ro2-related)
   - [Run Demo Nodes](#2run-demo-nodes)
   - [Run Turtlesim](#3run-turtlesim)
4. [Start Gazebo](#4start-gazebo)
5. [References](#references)
6. [License](#license)

---

# 1.Install
ros2-humble-install
```
sudo apt install git
git clone https://github.com/ChiaweiYu0824/ros2-humble-install.git
cd ros2-humble-install
chmod +x install_ros2_humble_gazebo.sh
./install_ros2_humble_gazebo.sh 
```
# 2.Build
Build workspace
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ros/ros_tutorials.git -b humble
cd ~/ros2_ws
colcon build
source install/setup.bash
```
# 3.Check
## Check RO2 related
Version
```
ros2 --version
```
Pkg
```
ros2 pkg executables turtlesim
```
## 2.Run Demo Nodes
### C++ Example
Publisher
```
ros2 run demo_nodes_cpp talker
```
Subscriber(Add new terminal)
```
ros2 run demo_nodes_cpp listener
```
### Python Example
Publisher
```
ros2 run demo_nodes_py talker
```
Subscriber(Add new terminal)
```
ros2 run demo_nodes_py listener
```
## 3.Run Turtlesim
Turtlesim Node
```
ros2 run turtlesim turtlesim_node
```
Turtle Teleop (Add new terminal)
```
ros2 run turtlesim turtle_teleop_key
```
# 4.Start Gazebo
Run Gazebo 
```
gazebo
```

---

## References
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/index.html)
- [Gazebo Documentation](https://gazebosim.org/home)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)

---

## License
This guide is released under the **MIT License**. You are free to use, modify, and distribute it with attribution.

---
