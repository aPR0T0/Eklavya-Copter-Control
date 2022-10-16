# Eklavya'22 Copter Control 

![](./assets/Copter-Control.jpg)

**Designing & Implementing an Optimal Control System of an Overactuated Hexacopter with Co-axial Tilt-Rotors for Efficient Omnidirectional Flight in Simulation(Gazebo) using ROS & Python. Learning about dynamics of a basic UAV and further implementing that knowledge in understanding the various dynamics of our system.**


## Demo

<img src="/assets/Demo.gif" width= "720">


<!-- TABLE OF CONTENTS -->
## Table of Contents

- [Project](#Eklavya-Copter-Control)
  - [Demo](#demo)
  - [Table of Contents](#table-of-contents)
  - [About The Project](#about-the-project)
    - [Tech Stack](#tech-stack)
    - [File Structure](#file-structure)
  - [Getting Started](#getting-started)
    - [Prerequisites and installlation](#prerequisites-and-installlation)
    - [Installation](#installation)
    - [Execution](#execution)
  - [Algorithm Flowchart](#algorithm-flowchart)
  - [Model and Implementation](#model-and-implementation)
  - [Future Work](#future-work)
  - [Contributors](#contributors)
  - [Acknowledgements and Resources](#acknowledgements-and-resources)
  - [License](#license)
  - [Citation](#citation)
<!--ABOUT THE PROJECT -->
## About The Project
Drone aviation is an emerging industry with possible applications in agriculture, healthcare, e-commerce as well as traffic control. We were interesting in getting first hand experience in Dynamics of a UAV to get a firm grasp on the principles needed to work with drones in the future.

Extending the maneuverability of unmanned areal vehicles promises to yield a considerable increase in the areas in which these systems can be used. Some such applications are the performance of more complicated inspection tasks and the generation of complex uninterrupted movements of an attached camera. In our project, we our designing and implementing a control system for a novel aerial platform that combines the advantages of existing multi-rotor systems with the agility of omnidirectionally controllable platforms. We are using a hexacopter with co-axial tiltable rotors(omav) allowing the system to decouple the control of position and orientation.

This project involves understanding key concepts of Dynamics of UAVs, Modern Robotics, Control Systems, ROS, Gazebo which is essential knowledge to understand and work in the field of Robotics

Our full project report can be found [here](./report/Project_Report_Copter_Control.pdf)

[Brief Presentation](./report/Copter-Control.pptx) can be found here

[Detailed Presentation](https://www.canva.com/design/DAFO6QGqbzk/BZ5_9Rk-LoEGTwhVJiBExA/view?utm_content=DAFO6QGqbzk&utm_campaign=designshare&utm_medium=link2&utm_source=sharebutton) can be found here

## Project workflow
- To learn about different control systems
- Understanding Dynamics of a UAV and of our system(omav)
- Design a Control System for our model(omav)which is a model of hexacopter modelled by ETH-Zürich
- Implementing the Control System in Simulation(Gazebo)

### Tech Stack

- [ROS Noetic](http://wiki.ros.org/noetic)
- [Gazebo](http://gazebosim.org/)
- [Python 3](https://www.python.org/downloads/)
- Modern Robotics
- Control Systems


### File Structure
```
👨‍💻Eklavya-Copter-Control
 ┣ 📂assets                               # reference gifs, images
 ┣ 📂Images
 ┣ 📂Records and Reports                  # Project Report & Presentation
 ┣ 📂simulation
 ┃ ┣ 📂rotors_comm                        # msg files for WindSpeed
 ┃ ┣ 📂rotos_description                  # All urdfs and meshes
 ┃ ┃ ┣ 📂meshes
 ┃ ┃ ┣ 📂urdf
 ┃ ┃ ┃ ┗ 🗃️omav.xacro                     # Drone Model
 ┃ ┃ ┣ 🗃️CMakeLists.txt
 ┃ ┃ ┗ 🗃️package.xml
 ┃ ┣ 📂rotors_gazebo                      # Launch Files, World Files & other Gazebo resources
 ┃ ┃ ┣ 📂scripts   
 ┃ ┃ ┃ ┣🗃️control_omav.py
 ┃ ┃ ┃ ┣🗃️pid_omav.py
 ┃ ┃ ┃ ┣🗃️force_desired.py
 ┃ ┃ ┃ ┣🗃️moment_desired.py
 ┃ ┃ ┃ ┣🗃️moment_force_allocation.py
 ┃ ┃ ┃ ┣🗃️speed.py
 ┃ ┃ ┃ ┗🗃️takeoff.py                      # Drone Testing Script
 ┃ ┃ ┣ 📂launch
 ┃ ┃ ┃ ┗ 🗃️mav.launch                     # Main Launch File
 ┃ ┃ ┣ 📂models
 ┃ ┃ ┣ 📂resource
 ┃ ┃ ┣ 📂worlds
 ┃ ┃ ┃ ┗ 🗃️basic.world                    # World used in Gazebo
 ┃ ┃ ┣ 🗃️CMakeLists.txt
 ┃ ┃ ┗ 🗃️package.xml
 ┃ ┗ 📂rotors_gazebo_plugins              # All Gazebo Plugins                       
 ┣ 🗃️LICENSE
 ┣ 🗃️Installations.md
 ┣ 🗃️Instructions.md
 ┗ 🗃️README.md
 ```

<!-- GETTING STARTED -->
## Getting Started

### Prerequisites and installlation
* [Ubuntu 20.04](https://ubuntu.com/download/desktop)
* [ROS Noetic](http://wiki.ros.org/noetic/Installation)
* [Gazebo Sim](http://gazebosim.org/)
* It is recommended to install Gazebo along with ROS and not seperately

### Installation

[Installation Guide](./assets/Installations.md "Installation")

### Execution
Open two terminal windows and run the following commands
- Terminal 1

```sh
source ~/catkin_ws/devel/setup.bash
roslaunch rotors_gazebo mav.launch mav_name:=omav
```
- Terminal 2
```sh
source ~/catkin_ws/devel/setup.bash
cd ~/Eklavya-Copter-Control/scripts
chmod +x .                      
python3 control_omav.py
```

<!--Flowchart -->

## Algorithm Flowchart 
Control System
![](./assets/Overall_flow.png)
Controller and Sensor Readings, Reference Data Flowchart
![](./assets/Basic_algo_flow.png)
Simplified code structure 
![](./assets/Simplified_code_struct.png)

<!-- Model And Implementation -->
## Model And Implementation

### Model  

![](./assets/Model.png)

### Gains (Tuning Parameters)

![](./assets/Gains.jpeg)

### Implementation


<img src="/assets/Copter-Control.gif" width= "720">


<!-- FUTURE WORK -->
## Future Work
Designing & Implementing an Optimal Control System of an Overactuated Hexa-copter with Co-axial Tilt-Rotors for Efficient Omnidirectional Flight in Simulation(Gazebo) using ROS & Python. Learning about the dynamics of a basic UAV and further implementing that knowledge in understanding the various dynamics of our system.

- [x] Designing Optimal Control System for Position Control using PID Algorithm
- [x] Designing Optimal Control System for Attitude Control
- [x] Implementing Control System with PID Tuning in Simulation(Gazebo) for reaching arbitrary altitude
- [x] Implementing Control System with PID Tuning in Simulation(Gazebo) for reaching Co-ordinates in free space
- [x] Implementing Control System with Tuning in Simulation(Gazebo) with decoupled Position and Orientation
- [ ] Improve Tuning, Control Algorithm for Efficient, Accurate, Stable and Fast Flight with decoupled Position and Orientation
- [ ] Achieve Stable Flight in presence of external disturbances like Wind,...
- [ ] Exploring different Control Systems & Approaches to improve performance of system
- [ ] Implement obstacle avoidance 

<!-- CONTRIBUTORS -->
## Contributors
* [Aryan Shah](https://github.com/ars-21)
* [Alqama Shaikh](https://github.com/aPR0T0)


<!-- ACKNOWLEDGEMENTS AND REFERENCES -->
## Acknowledgements and References
* [SRA VJTI](http://sra.vjti.info/) Eklavya 2022  
* [ETH-Zürich : ethz-asl/rotors_simulator Repository](https://github.com/ethz-asl/rotors_simulator) for the plugins as well as the model of the drone.
* [Tim Wescott](http://wescottdesign.com/articles/pid/pidWithoutAPhd.pdf) for the paper PID without PhD which was extremely illuminating for beginners in PID
* Our mentors [Sagar Chotalia](https://github.com/sagarchotalia), [Jash Shah](https://github.com/Jash-Shah) and [Ayush Kaura](https://github.com/Ayush-Kaura) for their guidance throughout the whole project
 
<!-- License -->
## License
[MIT License](https://opensource.org/licenses/MIT)

<!-- Citation -->
## Citation
```sh
@Inbook{Furrer2016,
author="Furrer, Fadri
and Burri, Michael
and Achtelik, Markus
and Siegwart, Roland",
editor="Koubaa, Anis",
chapter="RotorS---A Modular Gazebo MAV Simulator Framework",
title="Robot Operating System (ROS): The Complete Reference (Volume 1)",
year="2016",
publisher="Springer International Publishing",
address="Cham",
pages="595--625",
isbn="978-3-319-26054-9",
doi="10.1007/978-3-319-26054-9_23",
url="http://dx.doi.org/10.1007/978-3-319-26054-9_23"
}
```
