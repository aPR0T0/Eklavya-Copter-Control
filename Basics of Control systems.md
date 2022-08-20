# Overview
Everything around us working to make our lives easier is nothing but systems.
How they interact with the environment and what should be their behavior is decided by th control over the systems which we have. By combining real life parameters, system variables, and control of the user we can make a control system.

## Types of systems
- ### Non-linear :
    These are real form of systems which represent actual data of interaction with surrounding. These are high fidely, highly accurate form of systems and involves all non-linear factors, or one can say some real life unpredictable factors in the system.
    For Examaple: Consider a drone, here if we take in account the bending stress, strain, strength of the materials, environment correction, etc. while designing the control system then this will be known as a non-linear system. Here the accuracy of design is higher but the computational power required also increases drastically.
                
- ### Linear : 
    These are very ideal form of systems which don't really represent the real world scenario. But these systems are often used along side with the non-linear system to make a program faster and doesn't involve extra computation which is usually required for the non-linear type of systems.
    For Example: Again consider a drone, but we are not taking in account all the factors that we took previously. So, now the varaibles are less so is the accuracy, but it can be improved by just verification by the non-linear model whenever required.
## Control system for copter
We are using **PID controller** for our hexacopter. 
