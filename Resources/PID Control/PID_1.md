# PID CONTROL #1

- it is a type of control system
- simple, efficient and effective in a wide array of applications

> **PID**
    - Proportional
    - Integral
    - Derivative
![](./References/1_1.jpeg)

- each of these terms describe how the error term is treated prior to being summed and sent into the plant(process)

- The proportional term gives the maximum change and try to bring the bot
on the line
- But if we only give Proportional term it could lead to oscillations
- Hence Derivative Term is used which will give a curve which has reducing amplitude
- But there is a problem with this as once the bot becomes parallel to the line then with very less displacement then the bot will not center itself to the line
- Hence we need the Integral Term which will bring the bot to the center of the line.


> Block Diagram
![]()

- in the proportional path the error term is multiplied by a constant Kp
- in the integral path the error is multiplied by Ki and then integrated
- in the derivative path it's multiplied by Kd and then differentiated
- the three pads are then summed together to produce the controller output
- the three K terms are called gains and they can be adjusted or tuned to a particular plant with a defined set of requirements
- by changing these values you're adjusting how sensitive the system is to each of these different paths either the P I or D path




 let me explain what I mean
with a few plots here
let's see the error in the system is
changing over time like this red line in
the proportional path the output is the
error scaled by the game KP so you can
see here that when the error is large
the proportional path will produce a
large output when the error is zero the
output in the path is zero and when it's
negative the output is negative in the
integral path as the error moves over
time the integral will continually sum
it up and multiply it by the constant ki
in this plot it's easy to see that the
integral path is the area under the
curve where this blue section is
positive area in this green section here
is negative area now the integral path
is used to remove constant errors in a
control system since no matter how small
the constant error eventually the
summation of that error will be
significant enough to adjust the
controller output now in the derivative
path it's the rate of change of the
error that contributes to the output
signal when the change in error is
moving relatively slowly like it is at
the beginning hair then the derivative
path is small and the faster the error
changes the larger the derivative path
becomes now at this point you can just
sum up each of these three paths and
you've got the output of a PID
controller but you don't always need all
three paths you can remove a path
completely by setting its associated
gain to zero when you do this you
generally refer to the controller with
the letters of the path that are left
for example you can have a proportional
integral controller or P I if you set KD
to zero and just ap controller if KD and
ki are both zero so why would you
simplify the controller like that
why not just make the biggest best
controller you can with all paths intact
and super complicated well typically
when I'm designing a control law I tried
to make the logic as simple as I can
while still meeting all design
requirements I do this for several
reasons one is a simple controller is
easy to implement - a simple controller
is easy to tune test and troubleshoot
when there's problems and three a simple
controller is easy for other people to
understand which is important when you
work in a large project and interface
with other group
that have to buy into the control logic
for example a software or hardware team
that has to implement it so simple
controllers can save you time and money
over the life of the program as long as
they still meet your design requirements
and this is why despite having a lot of
really complicated control systems out
there
the majority of industry still uses PID
controllers or some variation now you're
probably thinking right now all right
you've given me the definition of a PID
controller but that really didn't help
that much because if you're like me I
don't usually understand something by
definition alone and I need some
examples for it to sink in but I want to
end this video here so that it doesn't
become too long that you lose interest
but next week I'm going to have a video
that runs through a thought exercise on
PID control and then show some of the
math to back up that exercise now I'm
going to link a video here in this box
once it's complete so you can just click
on through if this box is empty that
means I haven't finished it yet so if
you don't want to miss it you haven't


### References -

- [PID Control - A brief introduction](https://www.youtube.com/watch?v=UR0hOmjaHp0&list=PLUMWjy5jgHK20UW0yM22HYEUTMJfla7Mb)