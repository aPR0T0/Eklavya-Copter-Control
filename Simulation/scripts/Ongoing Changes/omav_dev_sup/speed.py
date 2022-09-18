from mav_msgs.msg import Actuators
def speed_assign( tilt_ang, ang_vel_rot):
    global t
    t = 0
    speed = Actuators()
    if ( t == 0 ):
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
        # print("Once")
        t += 1

    speed.angular_velocities[0] = ang_vel_rot[4]
    speed.angular_velocities[1] = ang_vel_rot[1]
    speed.angular_velocities[2] = ang_vel_rot[0]
    speed.angular_velocities[3] = ang_vel_rot[3]
    speed.angular_velocities[4] = ang_vel_rot[5]
    speed.angular_velocities[5] = ang_vel_rot[2]
    speed.angular_velocities[6] = ang_vel_rot[4]
    speed.angular_velocities[7] = ang_vel_rot[1]
    speed.angular_velocities[8] = ang_vel_rot[0]
    speed.angular_velocities[9] = ang_vel_rot[3]
    speed.angular_velocities[10] = ang_vel_rot[5]
    speed.angular_velocities[11] = ang_vel_rot[2]
    speed.angular_velocities[12] = tilt_ang[4]  
    speed.angular_velocities[13] = (tilt_ang[1])
    speed.angular_velocities[14] = tilt_ang[0]
    speed.angular_velocities[15] = (tilt_ang[3])
    speed.angular_velocities[16] = (tilt_ang[5])
    speed.angular_velocities[17] = tilt_ang[2]

    # Limiting the speeds to the permissible limits
    if (speed.angular_velocities[0] > 1700): speed.angular_velocities[0] = 1700
    if (speed.angular_velocities[1] > 1700): speed.angular_velocities[1] = 1700
    if (speed.angular_velocities[2] > 1700): speed.angular_velocities[2] = 1700
    if (speed.angular_velocities[3] > 1700): speed.angular_velocities[3] = 1700
    if (speed.angular_velocities[4] > 1700): speed.angular_velocities[4] = 1700
    if (speed.angular_velocities[5] > 1700): speed.angular_velocities[5] = 1700
    if (speed.angular_velocities[6] > 1700): speed.angular_velocities[6] = 1700
    if (speed.angular_velocities[7] > 1700): speed.angular_velocities[7] = 1700
    if (speed.angular_velocities[8] > 1700): speed.angular_velocities[8] = 1700
    if (speed.angular_velocities[9] > 1700): speed.angular_velocities[9] = 1700
    if (speed.angular_velocities[10] > 1700): speed.angular_velocities[10] = 1700
    if (speed.angular_velocities[11] > 1700): speed.angular_velocities[11] = 1700

    print(speed.angular_velocities)
    return(speed)