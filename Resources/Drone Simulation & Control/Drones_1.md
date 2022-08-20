# Drone Simulation and Control #1

![](./References/1_1.jpeg)

**Drones** have sophisticated control systems programmed into them that allow them to be stable and fly autonomously with very little human intervention.

**Propellers** are spun in precise ways to control a drone in 6 DOF(maybe different).


> Workflow of a Control Project :
- Requirements
- Understand the System
- Modelling
- Controller Design
- Testing

![](./References/1_2.jpeg)


> **Rotorcraft** - Rotating Wing Aircraft which generates lift using rotating wing

![](./References/1_3.jpeg)

Even though all are rotary wing vehicles, they all have different dynamics. Hence requiring different control systems.


> Types of Configurations :

- Non-changeable : Existing hardware to be used
- Changeable : can change sensors/actuators, to re-configuration


## Sensors :

1. **Ultrasound Sensor :**
- measure vertical distances
- sends out a high-frequency sound pulse and measures how long it takes for that pulse to bounce off the floor and return back to the sensor
- from the measured time distance between the floor and the drone can be calculated
- **Upto 13 feet altitude** as after that the reflected sound is too soft for the sensor to pick up

![](./References/1_4.jpeg)

2. **Camera :**
- captures images at 60 fps
- image processing technique called optical flow used to determine how objects are moving between one frame and the next
- this apparent motion the mini drone can estimate horizontal motion and speed

![](./References/1_5.jpeg)

3. **Pressure Sensor :**
- as the drone climbs in altitude the air pressure drops slightly
- this slight change in pressure is used to estimate the altitude

![](./References/1_6.jpeg)

4. **IMU - Inertial Measurement Unit :**
- 3-axis accelerometer which measures linear acceleration
- 3-axis gyroscope that measures angular rate
- from IMU and acceleration due to gravity we can estimate the drone's attitude relative to gravity and how fast it's rotating

![](./References/1_7.jpeg)


## Actuators :

- Colours of rotors are just there to indicate to the operator which way the drone is facing when they're flying it

![](./References/1_8.jpeg)

> Configuration & Spin Direction :

- X-Configuration
- Plus-Configuration
- Only difference will be which motors you send commands to when pitching and rolling

![](./References/1_27.jpeg)

- The most ingenious part of a quad copters motor configuration is the spin direction
- Opposing motors spin in the same direction as each other but opposite direction as the other pair
- This is necessary to make sure that thrust, roll, pitch and yaw can be commanded independently of each other
- That means that we can command one motion without affecting the others
- In reality complex fluid dynamics around the drone means that all motion is coupled together in some way but our feedback control system corrects this error

![](./References/1_28.jpeg)


## Control Problem


and eventually let our feedback control system correct for that error
now we can talk about the overview of the control problem
we have our plant
the drone itself
and we have our four actuators that inject forces and torques into the system
So the question is how do we inject the right inputs into our system
so that the output is the result we want
that is can we figure out a way to manipulate these four motors in a very precise way so that the drone can rotate
and maneuver around in three-dimensional space
To help us we have our set of sensors that we can use to directly or indirectly estimate the state of our mini drone
System states are things like angular position and rates altitude and horizontal velocity
and the states that were estimating depend on the control architecture and what we're trying to accomplish
We're gonna flesh that out in more detail throughout this series
Finally with knowledge of the state of the system and an understanding of what we want our mini-drone to do
we can develop a controller
Basically an algorithm that runs in software that takes in our set point and estimated state
and calculate those precise motor commands that will inject the necessary forces and torques
That's the whole problem
But as you might imagine coming up with that algorithm isn't straightforward for a quadcopter
The first thing we should notice is that this is an under actuated system
We only have four actuators
but we have six degrees of freedom
three translational directions up and down left and right forward and backwards
and three rotational directions roll pitch and yaw
and since we don't have an actuator for every motion
then we already know that some directions are uncontrollable at any given time
As an example our mini drone is not capable of moving left
at least not without first rotating in that direction
and the same goes for forward and backward motion as well
We're going to get around this under actuation problem by developing a control system that couples rotations and thrust to accomplish the overall goals
So now let's walk through how we generate thrust roll pitch and yaw with just four motors
and why the spin direction allows us to decouple one motion from the other
A motor produces thrust by spinning a propeller which pushes air down causing a reaction force that is up
If the motor is placed in a position that the force is applied through the center of gravity of an object
then that object will move in pure translation no rotation at all
and if the force of the thrust is exactly equal to and opposite the force of gravity
then the object will hover in place
A force at a distance from the center of gravity produces both a translational motion as well as a torque
or a rotating moment about the center of gravity
and if our motor is attached to the bar as it rotates
the torque will stay constant since the distance between the force and the center of gravity stays the same
but the force is now no longer always in the opposite direction of gravity
and therefore our bar will begin to move off the side and fall out of the sky
Now if there's a counter force on the opposite side of the center of gravity and each force is half of the force of gravity
then the object will again stay stationary because the torques and forces will cancel each other out
but our actuators don't generate purely force when it generates thrust
since it accomplishes thrust by rotating and torquing a propeller which has mass
our actuators are also generating a reaction torque that is in the opposite direction
and if both of our motors are spinning in the same direction
then the torque is doubled and our bar would start to rotate
now to counter this torque we could spend the two motors in opposite directions
and that would work just fine for two dimensions
but a bar with only two motors would not be able to generate torques in the third dimension
that is we wouldn't be able to roll this bar
So we add a second bar with two more motors to create our quadcopter
With this configuration we can hover by accelerating each motor until they each produce a force one fourth that of gravity
and as long as we have two counter-rotating motors
the torques from spinning the propellers will balance out and the drone will not spin
It doesn't matter where you put the counter-rotating motors
as long as there's two of them in one direction and two in the other direction
But quadcopter developers settled on a configuration with opposing motors spinning in the same direction
so there must be a reason for this and there is
It's because of how yaw or the flat spinning motion interacts with roll and pitch
To understand why this is true
let's look at how we would command yaw
We have counter rotating motors specifically
so that there is no yaw torque on the system with all motors spinning at the same speed
So it makes sense that if we want to spend the drone about the vertical axis or we want to have the vehicle yaw
then we need to create a yaw torque by slowing two motors down that are running in the same direction and speed the other two up
By slowing two motors down and speeding two up
appropriately we can keep the same total force throughout the maneuver
so that we're still hovering and counteracting gravity
but the summation of the motor torques is nonzero and the vehicle will spin
So in this way we can yaw without affecting thrust
Now let's see if yah effects roll and pitch
If the rotating motor pairs are on the same side
then slowing one pared-down and increasing the other pair will cause an imbalance of forces about the center of gravity
and the vehicle will either pitch or roll depending on which side the motor pairs are on
However if we separate the two motors and place them on opposite sides of the drone
then the forces will balance each other out
And this is why motor configuration and spin directions are so critical
We can now send commands to the four motors in such a way
that the vehicle will yaw but not roll pitch or change its thrust
Similarly we can look at roll and pitch
To roll we decrease one of the left/right pairs
and increase the other causing a rolling torque
And to pitch we decrease one of the front/back pairs and increase the other causing a pitching torque
Both of these motions would have no effect on yaw
since we're moving counter rotating motors in the same direction
and their yaw torque would continue to cancel each other out
Now to change thrust we need to increase or decrease all four motors simultaneously
In this way roll pitch yaw and thrust are the four directions that we have direct control over
and the commands to the motors would be a mix of the amount of thrust roll pitch and yaw required
As we now know we can command thrust by setting all four motors to the same speed
then we can create yaw by increasing two motors spending the same direction and decrease the other two
Pitch is created by increasing or decreasing the front motor pair
and then commanding the back pair in the opposite direction
And roll is the same but with the left right pair
This is our simple motor mixing algorithm that can convert between the intuitive roll pitch yaw and thrust and the less intuitive motor and speeds
Now as I said earlier
moving forwards backwards left and right our unactivated motions
and the way we get around that is by first rotating into an attitude where the thrust vector is partially in the gravity direction
partially in the direction of travel to accelerate the drone in that way
Now if we wanted to maintain altitude while we do this maneuver
then we would increase the thrust
so that the vertical component is still cancelling out the downward pull of gravity
All right
So we know that manipulating the four motors in specific ways will allow us to control the drone in 3d space
and we have a set of sensors that we can use to estimate the state of the system
and we have an onboard processor that can run our control logic
The control system development will ultimately be done in Simulink
where we're going to build and simulate the quadcopter model
tune the controller
test it in a closed loop simulation
and then finally automatically generate flight code that we're going to load into the onboard microcontroller on the parrot mini-drone
But the very next step is to figure out how we want to set up the control system architecture
and we're going to talk about that in the next video
So if you don't want to miss the next Tech Talk video
don't forget to subscribe to this channel
Also if you want to check out my channel control system lectures
I cover more control theory topics there as well
Thanks for watching and I'll see you next time