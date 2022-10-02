# Installation Instructions on Ubuntu 20.04 for ROS NOETIC
> **ETH-Zürich** : ethz-asl/rotors_simulator Repository

## Prerequisites

- Ubuntu 20.04 OS
- ROS Noetic Ninjemys (Note: We will be using Ros 1 and python3)
- Gazebo and Gazebo Ros Installation

### If not installed any follow the following links :

- [Ubuntu 20.04 LTS Installation](https://ubuntu.com/download/desktop "Ubuntu Installation")
- [Ros Noetic Ninjemys Installation (Ros 1) ](http://wiki.ros.org/noetic/Installation/Ubuntu "Ros Noetic Installation")
- [Gazebo Installation](https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros "Gazebo Installation Tutorial")



## 1. Install and initialize ROS kinetic desktop full, additional ROS packages, catkin-tools, and wstool :

(Commands being run in Ubuntu Terminal in Home Directory)

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-noetic-desktop-full ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink python3-wstool python3-catkin-tools protobuf-compiler libgoogle-glog-dev ros-noetic-control-toolbox ros-noetic-mavros
sudo rosdep init
rosdep update
source /opt/ros/noetic/setup.bash
```

* **If you don’t have `source /opt/ros/noetic/setup.bash` line in your bash file then run following commands**

    `gedit ~/.bashrc`

    Add following command at the end of your bash file `source /opt/ros/noetic/setup.bash`

    “SAVE” and then close the file

    Then run following command in terminal `source ~/.bashrc`

* **If you face a problem like:**
![Error faced](./assets/Installation_Dependencies.png "Error based on unmet dependencies")
    
    Then just before using `line 3` of the above block just type in `sudo apt install -f` (This is auto remove feature of ros of unwanted commands) after this the problem should be solved.


* **If you face a problem like:**
![Error faced](./assets/source_file_exists.png "Source File Already Exists")
    
    It is necessary to solve this error (But in few cases it might be required)

    Run `sudo rm <path mentioned in error>`, for example `sudo rm /etc/ros/rosdep/sources.list.d/20-default.list`

    Then Run `sudo rosdep init`

    Then Run `rosdep update`



## 2. Since we don’t have a workspace for the given project, we will make one :

(Commands being run in Ubuntu Terminal in <workspace_name>/src directory once workspace created)

```
mkdir -p ~/hexacopter_ws/src
cd ~/hexacopter_ws/src
catkin_init_workspace
wstool init
wget https://raw.githubusercontent.com/ethz-asl/rotors_simulator/master/rotors_hil.rosinstall
wstool merge rotors_hil.rosinstall
wstool update
```



## 3. Cloning Repository :

**Note the repository is to be cloned in the <workspace_name>/src directory**

(Note : Here we are using ssh key to clone repository, other methods can also be used. Also, recursive cloning is used so any linked components if any are not missed)

```
<<<<<<< HEAD
cd ~/hexacopter_ws/src
git clone https://github.com/aPR0T0/Eklavya-Copter-Control.git
```



## 4. Building Workspace with `python3_catkin_tools` (therefore you need `python3_catkin_tools`) :

(Commands being run in Ubuntu Terminal in <workspace_name> directory)

Checking/Installing `python3_catkin_tools` :

```
sudo apt-get update
sudo apt-get install python3-catkin-tools
```

Building Workspace :

```
cd ~/hexacopter_ws
catkin init
catkin build
```

If any errors occur you could run `catkin init` and `catkin build` commands again

**Note : There should be total 11 packages (maybe more) installed successfully for installation to be successful**



## 5. Add Sourcing to your `~/.bashrc` file :

```
echo "source ~/hexacopter_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```


### REFERENCES :
> **ETH-Zürich** : ethz-asl/rotors_simulator Repository

- [ethz-asl/rotors_simulator Repository](https://ubuntu.com/download/desktophttps://github.com/ethz-asl/rotors_simulator.git "ethz-asl/rotors_simulator")
=======
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc
```
>>>>>>> parent of de97693 (Some more instructions)
