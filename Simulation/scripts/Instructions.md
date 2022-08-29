# Errors Faced
In this section we are trying to cover nearly all the errors we have faced, what mistakes was there from our side and how did we tackle them.
 
## Launching the simulation
There are number of changes required to be made in the .launch files which are providied here [pdfs]((ResourcesandResearchpapers)/Rotors_Sim.pdf)

* Step 1 : `roslaunch rotors_gazebo mav_hovering_example.launch`
* Step 2 : Open new terminal 
* Step 3 : 
## Concept of rosmsg, rostopic, and rosnode

Consider a scenario when you are talking to your friend on your phone.
Here, you and your friend are the `nodes`. 
* rosnodes : So, these are basically the users or programs that publish or subscribe to any information
> ### How to know everything about your nodes?
> rosnode 

The network or line you are using for the communication is the `topic`, or you may even consider your conversations topic
* rostopic : So, It is just the channel through which you have to transfer the data
