
def speed_assign( tilt_ang, ang_vel_rot,speed,flag):
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
    i1 = 0
    for i1 in range(12):
        if (speed.angular_velocities[i1] < 640): speed.angular_velocities[i1] = 640

    print(speed.angular_velocities)
    return(speed)