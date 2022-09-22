import numpy as np

def speed_assign( tilt_ang, ang_vel_rot,speed,flag):

    global prev_ang_vel_rot, prev_tilt_ang

    if (flag == 0):
        prev_ang_vel_rot = np.zeros(6)
        prev_tilt_ang = np.zeros(6)


    for i in range(0, 6):
        if (ang_vel_rot[i] - prev_ang_vel_rot[i] > 1):
                ang_vel_rot[i] = prev_ang_vel_rot[i] + 1
        if (ang_vel_rot[i] - prev_ang_vel_rot[i] < -1):
                ang_vel_rot[i] = prev_ang_vel_rot[i] - 1

    for i in range(0, 6):
        if(ang_vel_rot[i] < 650):
            ang_vel_rot[i] = 650
        if(ang_vel_rot[i] > 750):
            ang_vel_rot[i] = 750

    for i in range(0, 6):
        speed.angular_velocities.append(ang_vel_rot[i])
    for i in range(0, 6):
        speed.angular_velocities.append(ang_vel_rot[i])

    for i in range(0, 6):
        speed.angular_velocities.append(tilt_ang[i])

    """
    speed.angular_velocities.append(ang_vel_rot[4])
    speed.angular_velocities.append(ang_vel_rot[1])
    speed.angular_velocities.append(ang_vel_rot[0])
    speed.angular_velocities.append(ang_vel_rot[3])
    speed.angular_velocities.append(ang_vel_rot[5])
    speed.angular_velocities.append(ang_vel_rot[2])
    speed.angular_velocities.append(ang_vel_rot[4])
    speed.angular_velocities.append(ang_vel_rot[1])
    speed.angular_velocities.append(ang_vel_rot[0])
    speed.angular_velocities.append(ang_vel_rot[3])
    speed.angular_velocities.append(ang_vel_rot[5])
    speed.angular_velocities.append(ang_vel_rot[2])
    speed.angular_velocities.append(tilt_ang[4])
    speed.angular_velocities.append(tilt_ang[1])
    speed.angular_velocities.append(tilt_ang[0])
    speed.angular_velocities.append(tilt_ang[3])
    speed.angular_velocities.append(tilt_ang[5])
    speed.angular_velocities.append(tilt_ang[2])
    
    
    if (flag != 0):
        t = 0
        for t in range(12):
            speed.angular_velocities[t] = ang_vel_rot[t]
        t = 0
        for t in range(6):
            speed.angular_velocities[12+t] = tilt_ang[t]  

    # Limiting the speeds to the permissible limits
    i1 = 0
    for i1 in range(12):
        if (speed.angular_velocities[i1] > 900): speed.angular_velocities[i1] = 900
        print("HI")
    i1 = 0
    for i1 in range(12):
        if (speed.angular_velocities[i1] < 640): speed.angular_velocities[i1] = 640
    """



    prev_ang_vel_rot = ang_vel_rot
    prev_tilt_ang = tilt_ang

    print(speed.angular_velocities)
    return(speed)