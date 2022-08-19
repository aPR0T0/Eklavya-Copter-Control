# Installation Instructions on Ubuntu
1. Install and initialize ROS noetic desktop full, additional ROS packages, catkin-tools, and wstool:

```
    1. sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
    2. wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
    3. sudo apt-get install ros-noetic-desktop-full ros-noetic-joy ros-noetic-octomap-ros python3-wstool python3-catkin-tools protobuf-compiler

    4. gedit ~/.bashrc
```
* If you face a problem like
![Error faced]()
###  After gedit you should add 'source /opt/ros/noetic/setup.bash' at the end of the file and then save it

2. If you don't have ROS workspace yet you can do so by
```
    1. mkdir -p workspace/src
    2. cd workspace
    3. catkin_make
```
Iff catkin_make doesn't work try using catkin_make_isolated instead.

3. Add source to your .bashrc file
```
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc
```
## Get the simulator and additional dependencies
```
    cd ~/workspace/src
    git clone git@github.com:ethz-asl/rotors_simulator.git
    git clone git@github.com:ethz-asl/mav_comm.git
```