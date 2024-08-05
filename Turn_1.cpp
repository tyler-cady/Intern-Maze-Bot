void Turn(bool R_L, int degree, int turn_speed) 
{
    // Get initial yaw angle
    float initial_yaw = MPU_getX();
    float target_yaw;
    
    if (R_L){
        // Right turn
        target_yaw = initial_yaw + degree;
        if (target_yaw > 180) target_yaw -= 360;  // Normalize to -180 to 180
    }
    else{
        // Left turn
        target_yaw = initial_yaw - degree;
        if (target_yaw < -180) target_yaw += 360;  // Normalize to -180 to 180
    }

    // Begin turning
    if (R_L){
        setSpeed(turn_speed, PWMright1);
        setSpeed(0, PWMright2);
        setSpeed(0, PWMleft1);
        setSpeed(turn_speed, PWMleft2);
    }
    else{
        setSpeed(0, PWMright1);
        setSpeed(turn_speed, PWMright2);
        setSpeed(turn_speed, PWMleft1);
        setSpeed(0, PWMleft2);
    }

    // Continue turning until the target yaw is reached
    while (true) {
        float current_yaw = MPU_getX();
        if (R_L) {
            if (initial_yaw <= target_yaw) {
                if (current_yaw >= target_yaw || current_yaw <= initial_yaw - 180) break;
            } else {
                if (current_yaw >= target_yaw && current_yaw <= initial_yaw + 180) break;
            }
        } else {
            if (initial_yaw >= target_yaw) {
                if (current_yaw <= target_yaw || current_yaw >= initial_yaw + 180) break;
            } else {
                if (current_yaw <= target_yaw && current_yaw >= initial_yaw - 180) break;
            }
        }
    }

    // Stop the motors
    motors_stop(2);
    resetEncoders();  // reset encoders after turn
}
