# PID CONTROL #4

## TUNING

- If you can, hook your system up to some test equipment, or write in some debug code to allow you to look at the appropriate variables. If your system is slow enough you can just spit the appropriate variables out on a serial port and graph them with a spreadsheet. If you are tuning something motorized you may be able to just watch it’s behavior. Ideally, you want to be able to look at the drive output and the plant output, to get graphs. In addition, you also want to apply a changing command—preferably a square-wave—to your system

- Once setup is ready, start by setting all gains to zero
- start by adjusting your differential gain


### 1. Adjusting Derivative Gain

- The way the controller is coded you cannot use derivative control alone
- we need to set your proportional gain to a value that’s low enough to prevent oscillation, or at least so that the system is oscillating much more slowly than you want it to react when it is tuned
- Check to see how the system works. If it oscillates with proportional gain you should be able to cure it with differential gain
- Now put in some derivative gain. Start with about 100 times more derivative gain than proportional gain
- Watch your drive signal while you stimulate the system
- If the system oscillates under derivative control when it did not oscillate under proportional control alone, or if the system oscillates much faster when you dial in some derivative gain, back the derivative gain off by factors of two until it stops
- If you did not need to decrease the derivative gain to make the system stop oscillating, start increasing it gain until you do see oscillation, excessive noise or excessive (more than 50%) overshoot on the drive or plant output
- Note that the oscillation from too much derivative gain is much faster than the oscillation from not enough, or from too much proportional gain
- push the gain up until the system is on the verge of oscillation, then back the gain off by a factor of 2 or 4
- Make sure the drive signal still looks good
- At this point your system will probably be settling out very sluggishly, so its time to tune the proportional and integral gains


### 2. Adjusting Proportional Gain

- If you have nonzero derivative gain in your system, a good starting value for the proportional gain is 1 / 100 of the derivative gain value. This may cause the system to oscillate, it may leave the system very sluggish, but it should be a starting point
- If you are not using derivative action in the system, find a starting value for the proportional gain. In most control systems, a gain of between 1 and 100 is a good point to start. With this initial value your system will probably either show terribly slow performance or it will oscillate
- If you see oscillation drop the proportional gain by factors of 8 or 10 until the oscillation stops
- If you don’t see oscillation, increase the proportional gain by factors of 8 or 10 until you start seeing oscillation or excessive overshoot
- Once you are close, fine tune the proportional gain by factors of two until you see oscillation, then back the gain off by a factor of two or four.


### 3. Adjusting Integrator Gain

- Once you have your proportional gain set, start adjusting integral gain
- If you are using derivative gain, a good starting value for the integrator gain is to set it smaller than the proportional gain by the same ratio as proportional gain to derivative gain
- if you are not using derivative gain, a good starting value for the integrator gain will be around 1 / 100 of the proportional gain
- If you see oscillation, decrease the integrator gain by steps of 8 or 10 until the oscillation goes away
- If you don’t see oscillation, increase the integrator gain by steps of 8 or ten until you do
- now 
- now try to find the gain where the system just breaks into oscillation, and then back the gain off by a factor of 2 or 4


### IMPORTANT 

- When you have gone through this sequence exactly once: find the derivative gain, find the proportional gain, and then find the integrator gain, you must stop. Do not go back and tweak the gains


### Problem 1 - Doing Math (Computational Time)

- we normally should not do computations in floating point. In fact, even at this writing (spring of 2016)
- processors that can do fast double-precision floating point are much costlier
- Most embedded processors can do integer arithmetic much faster than they can do floating point, and the tradeoff between the development time to use fixed-point math, and the savings in processor loading, is often a good one
- Unless you are working on a project with very critical performance parameters you can often get by with control gains that are within a factor of two of the "correct" value. This means that—if you are doing your computations with integer math—you can do all your "multiplies" with shifts. With smaller processors that do not do multiplication in hardware, this can lead to a dramatic decrease in the processor time taken for each iteration of the control loop. This, in turn, will lead to a matching increase in performance, or a decrease in processor performance requirements


### Problem 2 - Determining Sampling Rates

- If your sampling rate is too low you may not be able to achieve the performance you want, because of the added delay of the sampling
- If your sampling rate is too high you will create problems with noise in your differentiator and overflow in your integrator
- The rule of thumb for digital control systems is that the sample time should be between 1/10th and 1/100th of the desired system settling time
- system settling time is the amount of time from the moment that the drive comes out of saturation until the control system has effectively settled out
- You should treat the sampling rate as a flexible quantity
- Anything that might make the control problem more difficult would indicate that you should raise the sampling rate
- Factors such as having a difficult plant to control, or needing differential control, or needing very precise control would all indicate raising the sampling rate



## References

- [PID_Without_A_Phd](https://www.wescottdesign.com/articles/pid/pidWithoutAPhd.pdf)
- If this link doesn't work use [PID_Without_A_Phd](./References/pidWithoutAPhd.pdf)