# ROS 2 Humble Installation & Verification Guide

# 1.Install
1.ros2-humble-install
```
sudo apt install git
git clone https://github.com/ChiaweiYu0824/ros2-humble-install.git
cd ros2-humble-install
chmod +x install_ros2_humble_gazebo.sh
./install_ros2_humble_gazebo.sh 
```
# 2.Build
1.Build workspace
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
1.Version
```
ros2 --version
```
2.Pkg
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
