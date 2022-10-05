
def speed_assign( tilt_ang, ang_vel_rot,speed,flag):
    t = 0
    for t in range(12):
        if (ang_vel_rot[t] + 5 >= speed.angular_velocities[t]):
            speed.angular_velocities[t] = speed.angular_velocities[t] + 5
        elif (ang_vel_rot[t] - 5 <= speed.angular_velocities[t]):
            speed.angular_velocities[t] = speed.angular_velocities[t] - 5
        else:
            speed.angular_velocities[t] = ang_vel_rot[t]
    t = 0
    for t in range(6):
        speed.angular_velocities[12+t] = tilt_ang[t]  

    i1 = 0
    for i1 in range(12):
        if (speed.angular_velocities[i1] < 640): speed.angular_velocities[i1] = 640
    
    print(speed.angular_velocities)
    return(speed)