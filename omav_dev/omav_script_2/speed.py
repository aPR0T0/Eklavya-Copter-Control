import numpy as np
def speed_assign( tilt_ang, ang_vel_rot,speed,flag):
    global control_term
    speed.angular_velocities.append(0)
    speed.angular_velocities.append(0)
    speed.angular_velocities.append(0)
    speed.angular_velocities.append(0)
    speed.angular_velocities.append(0)
    speed.angular_velocities.append(0)
    speed.angular_velocities.append(0)
    speed.angular_velocities.append(0)
    speed.angular_velocities.append(0)
    speed.angular_velocities.append(0)
    speed.angular_velocities.append(0)
    speed.angular_velocities.append(0)
    speed.angular_velocities.append(0)
    speed.angular_velocities.append(0)
    speed.angular_velocities.append(0)
    speed.angular_velocities.append(0)
    speed.angular_velocities.append(0)
    speed.angular_velocities.append(0)

    if (flag != 0):
        t = 0
        control_flag = 0
        for t in range(6):
            if (ang_vel_rot[t] > 900):
                control_flag += 1
                if (speed.angular_velocities[t] > ang_vel_rot[t]):
                    speed.angular_velocities[t] = speed.angular_velocities[t] - (speed.angular_velocities[t]/ang_vel_rot[t])*0.15
            if (control_flag != 0) :
                if (speed.angular_velocities[t] <= (900) and ang_vel_rot[t] > (speed.angular_velocities[t] + 10)):
                    speed.angular_velocities[t] = speed.angular_velocities[t] + (ang_vel_rot[t]/speed.angular_velocities[t])*0.15
            else:
                speed.angular_velocities[t] = ang_vel_rot[t]
        t = 0
        control_flag = 0
        for t in range(6):
            if (ang_vel_rot[t] > 900):
                control_flag += 1
                if (speed.angular_velocities[6+t] > ang_vel_rot[t]):
                    speed.angular_velocities[6+t] = speed.angular_velocities[6+t] - (speed.angular_velocities[6+t]/ang_vel_rot[t])*0.2
            if (control_flag != 0):
                if (speed.angular_velocities[6+t] <= (870) and ang_vel_rot[t] > (speed.angular_velocities[6+t] + 10)):
                    speed.angular_velocities[6+t] = speed.angular_velocities[6+t] + (ang_vel_rot[t]/speed.angular_velocities[6+t])*0.2
            else:
                speed.angular_velocities[6+t] = ang_vel_rot[t]
        t = 0
        for t in range(6):
            speed.angular_velocities[12+t] = tilt_ang[t]  

    # Limiting the speeds to the permissible limits
    i1 = 0
    for i1 in range(12):
        if (speed.angular_velocities[i1] > 900): speed.angular_velocities[i1] = 900
    i1 = 0
    for i1 in range(12):
        if (speed.angular_velocities[i1] < 640): speed.angular_velocities[i1] = 640
    
    print(speed.angular_velocities)
    return(speed)