# Drone Simulation and Control #3

![](./References/3_1.png)

> **Steps Done -**
    - How quad-copters generate motion using their 4 propellers
    - A Control System Architecture which is capable of getting drone to hover

> **Steps Required to be Done -**
    - Code Control Logic (FLIGHT CODE) - in a way that we can put it on the mini drone
    - Tune and Tweak Flight Code - until the hover performance meets requirements - use MODEL-BASED DESIGN to achieve this goal which is to use a realistic model of the quad-copter in the environment to design our flight code and simulate the results
    - Fly Drone!!!

![](./References/3_2.png)

![](./References/3_3.png)



## Flight Control Software -

- it is just a small part of the overall flight code that will exist on the drone
- there's code to operate and interface the sensors and process their measurements
- there's code to manage the batteries, the Bluetooth interface and the LEDs
- there's code to manage the motor speeds and so on 

![](./References/3_4.png)

### 2 Ways to Implement the Flight Controller -

![](./References/3_5.png)

1. Option 1 - **Write C code by hand** -

- write the C code by hand and then compile the entire flight code with fewer changes to the flight controller and then finally load the compiled code to the mini drone

> Reasonable approach to creating flight code but it has a few **drawbacks** when developing feedback control software 
    - with this method we don't have an easy way to tune the controllers, except by tweaking them on the hardware
    - we could develop our own tuning tools and a model of the system, and then simulate how it would behave that way
    - but designing a modeling control systems in C code makes it harder to explain the architecture to other people and it's more difficult to understand how changes impact the whole system than it is with just graphical methods 
    
2. Option 2 - **Graphical Method** -

- describing the flight controller graphically using block diagrams
- we will develop the flight controller in Simulink
- then Auto code it into C code, where if we wanted to we can make changes manually 
- then compile that C code and load it onto the mini drone

> **Advantages -**
    - Simulink code is easier to understand 
    - additionally we can build a model of the drone and the environment and Simulink so that we can simulate the performance of our flight controller and use existing tools to tune the controller gains

> Now we won't need to worry about writing most of the flight code because we're going to use the Simulink Support Package for Parrot Mini-drones to program our custom flight control software 
    - this package loads new flight firmware on the vehicle in a way that keeps all of the normal operating functions of the drone in place but lets us replace the control portion of the firmware as long as we keep the same input and output signals intact 
    - then when we program the mini drone through Simulink 
    - any code we write will be placed in the correct spot and can interface with the rest of the mini drone firmware

![](./References/3_6.png)

    - design a Simulink model that takes external commands and the measured sensor readings as inputs
    - and outputs the commanded motor speeds along with the safety flag that will shut the mini drone down when it's set
    - this safety flag is our protection in case our code causes the drone to become uncontrollable/perform dangerous/non-desireable maneuvers
    - then we can build the C code from that model and fly the actual drone with that software to test the drone



## Sub-systems of Control System -

![](./References/3_7.png)

1. Controller -

- the flight control system with the developed architecture
- software algorithm


2. State Estimator -

- additional logic used to convert all of the measured states coming from the sensors into the control States needed by the controller
- Eg - air pressure sensor measures the air pressure reading 
        - this cannot be passed into our flight control system as we're not trying to control the drone to a particular pressure but we're trying to control an altitude 
        - hence,we require to convert from air pressure to altitude 


3. Fault Protection -

- logic that determines whether or not to set the shutdown flag 
- required as we'd run the risk of the drone flying away or causing harm to nearby observers 
- we could check for things like the rotation rate of the drone is above some threshold or the position or altitude is outside some boundary set
- relatively easy to develop code and can really save us from damaging the hardware or other people 


4. Data Logging -

- all of the firmware that exists on the mini drone records data that we have access to during and after the flight 
- since the mini drone doesn't know about the software that we've written 
- requirement to make sure that we have data logging set for the variables that were interested
- using Simulink can easily create logic
that will store data as a **.mat** file locally on the drone and then we can download it to MATLAB after the flight

> These are the 4 main subsystems that we need to develop in Simulink in order to have safe and functioning flight code



## Building Flight Code using Simulink -

> Note : we will not be using Simulink to make controllers, this is for understanding purposes

- Aerospace Blockset in MATLAB has a quadcopter project based on the parrot mini-drone, hence no need to write code from scratch, we're going to use this
- we understand how Simulink code matches the Control Architecture designed by us
- it also helps in state estimation data logging and fault protection
- following is a complete Simulink Model which also includes additional logic than our requirement

![](./References/3_8.png)

- the model has several subsystems like FCS, Airframe, Environment, and so on
- it is a classic feedback control system
- Signal Editor is generating the reference signals or the set points that we want the drone to follow
- flight control system block is where the error term is generated and the PID controllers live, this is the flight code block that gets auto coded and loaded onto the mini drone 
- the outputs from the FCS block are the motor commands that are sent to the rotors
- visualization block just plots the signals and runs the 3d visualizer and it's outside of our feedback loop
- environment block that models things like gravity vector and air pressure for the plant and the sensors
- the sensor model block which simulates the noise and dynamics of the force sensors that are on the mini drone

![](./References/3_9.png)

### FCS Block - 

- FCS block is the flight code and everything else is part of the model used for the simulation
- third input is the software as written makes use of the camera and image processing to help with precision landing 

> Following is Flight Control System Diagram

![](./References/3_10.png)

> Following is FCS Block

![](./References/3_11.png)

- the controller subsystem takes the reference signal and compares it to the estimated states to get the error signals
- this error is then fed into several PID controllers to ultimately generate the required motor commands

> Following is The Subsystem Block

![](./References/3_12.png)

- additional logic allows us to command special takeoff behaviors as well as control the roll and pitch angles directly for landing


> Controller for our basic requirement (Hover Maneuver) -

![](./References/3_13.png)

- XY position outer loop controller which is feeding into the roll and pitch inner loop controller
- independent of those two we have the yaw and altitude controllers
- 6 PID controllers that work together to control the position and orientation of the mini drone

> Altitude Controller -










### References -

- [Drone Simulation and Control, Part 3: How to Build the Flight Code](https://www.youtube.com/watch?v=3Gtb5Eq1Lvk&list=PLn8PRpmsu08oOLBVYYIwwN_nvuyUqEjrj&index=4)
