
def speed_assign( tilt_ang, ang_vel_rot,speed):
    t = 0
    for t in range(12):
        speed.angular_velocities[t] = ang_vel_rot[t]
    t = 0
    for t in range(6):
        speed.angular_velocities[12+t] = tilt_ang[t]  
    
    print(speed.angular_velocities)
    return(speed)