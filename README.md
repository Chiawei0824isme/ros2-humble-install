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
# 3.Check RO2 related
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
Subscriber
```
ros2 run demo_nodes_cpp listener
```
### Python Example
Publisher
```
ros2 run demo_nodes_py talker
```
Subscriber
```
ros2 run demo_nodes_py listener
```
# 4.Gazebo
```
gazebo
```

ROS2 humble LINK:https://docs.ros.org/en/humble/index.html
