
def speed_assign( tilt_ang, ang_vel_rot,speed,flag):
    t = 0
    for t in range(6):
        speed.angular_velocities[t] = ang_vel_rot[t]
    t = 0
    for t in range(6):
        speed.angular_velocities[6+t] = ang_vel_rot[t]
    t = 0
    for t in range(6):
        speed.angular_velocities[12+t] = tilt_ang[t]  

    # Limiting the speeds to the permissible limits
    i1 = 0
    for i1 in range(12):
        if (speed.angular_velocities[i1] > 800): speed.angular_velocities[i1] = 800
    i1 = 0
    for i1 in range(12):
        if (speed.angular_velocities[i1] < 640): speed.angular_velocities[i1] = 640
    
    print(speed.angular_velocities)
    return(speed)