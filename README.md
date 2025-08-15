<div align="center">
  <div style="display: flex; justify-content: center; align-items: center; gap: 20px; flex-wrap: wrap;">
    <img src="https://classic.gazebosim.org/assets/gazebo_vert-af0a0ada204b42b6daca54e98766979e45e011ea22347ffe90580458476d26d6.png" alt="Gazebo" height="180" style="background-color: white; padding: 10px;">
    <img src="https://docs.ros.org/en/humble/_static/humble-small.png" alt="ROS 2 Humble" height="180">
    <img src="https://raw.githubusercontent.com/ros-visualization/rviz/kinetic-devel/images/splash.png" alt="RViz2" height="180">
  </div>
</div>

# ROS 2 Humble Installation & Verification Guide
![Build Status](https://img.shields.io/badge/build-passing-brightgreen)
![License](https://img.shields.io/badge/license-MIT-blue)
![ROS 2 Version](https://img.shields.io/badge/ROS2-Humble-orange)

# Table of Contents
1. [Install](#1install)
2. [Build Workspace](#2build)
3. [Check](#3check)
   - [Check ROS 2 Related](#1check-ros2-related)
   - [Run Demo Nodes](#2run-demo-nodes)
   - [Run Turtlesim](#3run-turtlesim)
4. [Start Gazebo](#4start-gazebo)
5. [References](#references)
6. [License](#license)
7. [Disclaimer & Copyright](#disclaimer--copyright)


---

# 1.Install
ros2-humble-install
```
wget -c https://raw.githubusercontent.com/ChiaweiYu0824/ros2-humble-install/main/install_ros2_.sh && chmod +x ./install_ros2_.sh && ./install_ros2_.sh
```
# 2.Build
Build workspace
```
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ros/ros_tutorials.git -b humble
cd ..
rosdep install -i --from-path src --rosdistro humble -y
colcon build
ls
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
# 3.Check
## 1.Check ROS2 related
Version
```
echo $ROS_DISTRO
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
Launch Gazebo Demo World
```
gazebo worlds/rubble.world
```
---

## References
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/index.html)
- [RViz2](https://docs.ros.org/en/rolling/p/rviz2/__links.html)
- [Gazebo Documentation](https://gazebosim.org/home)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)

## License
>  This guide is released under the **MIT License**. You are free to use, modify, and distribute it with attribution.

## Disclaimer & Copyright

> **Disclaimer**: This guide is provided "as is" for educational and reference purposes only. The author makes no warranties, express or implied, and disclaims all liability for any damages or losses arising from the use of this guide. Users assume all risks associated with using this guide.

> **Copyright Notice**: This guide is based on official ROS 2 documentation and open-source community resources, following the open-source spirit for technical learning and communication purposes. All referenced content is properly attributed with appropriate links to original sources.

> **Logos**: Property of respective organizations, used for educational purposes. 
