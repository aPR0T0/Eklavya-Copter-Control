# Eklavya-Copter-Control

Controlling a hexacopter with coaxial tilt rotors in simulation, understanding the various dynamics of such a system and creating a controller that will enable a stable positioning system with multiple functionalities.

<!-- TABLE OF CONTENTS -->
## Table of Contents

- [Project](#Eklavya-Copter-Control)
  - [Table of Contents](#table-of-contents)
  - [About The Project](#about-the-project)
    - [Tech Stack](#tech-stack)
    - [File Structure](#file-structure)
  - [Getting Started](#getting-started)
    - [Prerequisites and installlation](#prerequisites-and-installlation)
    - [Installation](#installation)
    - [Execution](#execution)
  - [Algorithm Flowchart](#algorithm-flowchart)
  - [Results and Demo](#results-and-demo)
  - [Future Work](#future-work)
  - [Contributors](#contributors)
  - [Acknowledgements and Resources](#acknowledgements-and-resources)
  - [License](#license)

<!--ABOUT THE PROJECT -->
## About The Project
Drone aviation is an emerging industry. With possibilities for its applications in agriculture, healthcare, e-commerce as well as traffic control. We wanted to get first hand experience with how a drone is designed as well as how it flies to get a firm grasp on the principles needed to work with drones in the future.  
Our full project report can be found [here]()

## Project workflow
- To learn about different control systems and make gazebo simulation of a modelled hexacopter by ETH-Zürich
- Make the control system
- Implementation

### Tech Stack

- [ROS Noetic](http://wiki.ros.org/noetic)
- [Gazebo](http://gazebosim.org/)
- [Python 3](https://www.python.org/downloads/)


### File Structure
```
📦Eklavya-Copter-Control
 ┣ 📂Images                                 #contains gifs, videos and images of the results          
 ┣ 📂Resources and Research papers          #Contains all the research papers that we included for our case study
 ┣ 📂simulation
 ┃ ┣ 📂rotors_gazebo
 ┃ ┃ ┗ 📂launch                             #launch files
 ┃ ┃ ┃ ┗ 📜mav.launch                       #There are other launch files too but this is basic
 ┃ ┃ ┣ 📂models                             #files and meshes used to render the model
 ┃ ┃ ┣ 📂scripts                            #python programs used to run the drone   
 ┃ ┃ ┃ ┣📜control_omav.py                   #controller which initializes the controller node
 ┃ ┃ ┃ ┣📜pid_omav.py                       #contains the pid term calculations
 ┃ ┃ ┃ ┣📜force_desired.py                  #gets force the copter needs from inertial frame and then trasforms it to body frame
 ┃ ┃ ┃ ┣📜moment_desired.py                 #gets moment desired in body frame using some cool quaternions┗
 ┃ ┃ ┃ ┣📜moment_force_allocation.py        #recieves both force and moments and blend them together smoothly
 ┃ ┃ ┃ ┣📜speed.py                          #Co-axial rotors need this for speed distribution 
 ┃ ┃ ┃ ┗📜takeoff.py                        #Simple test file for rookies  
 ┃ ┃ ┣ 📂worlds                             #world files
 ┃ ┃ ┃ ┗ 📜basic.world
 ┃ ┃ ┣ 📜CMakeLists.txt
 ┃ ┃ ┗ 📜package.xml
 ┃ ┗ 📂
 ┣📂Some Basic Concepts
 ┣📂pid_sliders
 ┗📂drone      

 ```

<!-- GETTING STARTED -->
## Getting Started

### Prerequisites and installlation
* Tested on [Ubuntu 20.04](https://ubuntu.com/download/desktop)
* [ROS Noetic](http://wiki.ros.org/noetic/Installation)
* [Gazebo Sim](http://gazebosim.org/)
* Do visit these websites for the installation steps of the above mentioned software. It is recommended to install Gazebo along with ROS and not seperately

### Installation

[Installation Guide](./Installations.md "Installation")

### Execution
Open two terminal windows and run the following commands
- Terminal 1

```
source ~/catkin_ws/devel/setup.bash
roslaunch rotors_gazebo mav.launch mav_name:=omav
```
- Terminal 2
```
source ~/catkin_ws/devel/setup.bash
cd ~/Eklavya-Copter-Control/simulation/rotors_gazebo/scripts
chmod +x .                      
python3 control_omav.py
```


<img src="#" width="640" height="480" />


<!--Flowchart -->
## Algorithm Flowchart
Simplified code structure  
<img src="#">

<!-- RESULTS AND DEMO -->
## Results and Demo


Drone at start of the program:  

<img src="#">

Drone when target co-ordinates are given in control_omav.py:  




#





<!-- FUTURE WORK -->
## Future Work
- [x] Create a control system for the drone using PID
- [x] Stabilise the Roll, Pitch and Yaw of the Drone 
- [x] Get the drone to fly at any arbitrary altitude
- [x] Have the drone fly to given co-ordinates and stabilise itself
- [ ] To make copter fly with an arbitary orientation at a given position
- [ ] Implement obstacle avoidance 

<!-- CONTRIBUTORS -->
## Contributors
* [Alqama Shaikh](https://github.com/toshan-luktuke)
* [Aryan Shah](https://github.com/Jash-Shah)


<!-- ACKNOWLEDGEMENTS AND REFERENCES -->
## Acknowledgements and Resources
* [SRA VJTI](http://sra.vjti.info/) Eklavya 2022  
* [E-Yantra IIT-B](https://new.e-yantra.org/) for the plugins as well as the model of the drone. 
* [Nishanth Rao](https://github.com/NishanthARao/ROS-Quadcopter-Simulation) for the template of the PID controllers
* [Tim Wescott](http://wescottdesign.com/articles/pid/pidWithoutAPhd.pdf) for the paper PID without PhD which was extremely illuminating for beginners in PID
* Our mentors [Jash Shah](https://github.com/Jash-Shah), [Sagar Chotalia](https://github.com/kart1802) and [Ayush Kaura](https://github.com/dhruvi29) for their guidance throughout the whole project

<!-- -->
## License
[MIT License](https://opensource.org/licenses/MIT)
