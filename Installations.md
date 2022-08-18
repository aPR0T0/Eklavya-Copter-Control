# Installation Instructions on Ubuntu
1. Install and initialize ROS kinetic desktop full, additional ROS packages, catkin-tools, and wstool:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full ros-kinetic-joy ros-kinetic-octomap-ros ros-kinetic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-kinetic-control-toolbox ros-kinetic-mavros
sudo rosdep init
rosdep update
source /opt/ros/kinetic/setup.bash
```
2. If you don't have ROS workspace yet you can do so by
```
 mkdir -p ~/catkin_ws/src
 cd ~/catkin_ws/src
 catkin_init_workspace  # initialize your catkin workspace
 wstool init
 wget https://raw.githubusercontent.com/ethz-asl/rotors_simulator/master/rotors_hil.rosinstall
 wstool merge rotors_hil.rosinstall
 wstool update
 
 ```