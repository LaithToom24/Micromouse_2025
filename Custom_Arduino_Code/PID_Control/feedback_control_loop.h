/*
void set_velocity(float vel){
  // SETTING RIGHT MOTOR VELOCITY 

  // Calculating right speed
  right_velocity_prev = right_velocity;
  unfiltered_right_vel_prev =  unfiltered_right_vel; 
  total_rotations_right = right_count / ((float)counts_per_rev * gear_ratio);
  if (digitalRead(MOTOR_RIGHT_DIR))
    unfiltered_right_vel = fabs(1000.0f * total_rotations_right * wheel_circumference / dtime);
  else
    unfiltered_right_vel = -fabs(1000.0f * total_rotations_right * wheel_circumference / dtime);

  // Calculating error between target and actual velocity
  float error = vel - right_velocity;

  // Calculating integral term
  float integral = net_right_error + (error * dtime/1000.0f);
  float derivative = 1000.0f * (error - last_right_error) / dtime;

  net_right_error = integral;
  last_right_error = error;

  control_signal_right_prev = control_signal_right;
  unfiltered_control_signal_right_prev = unfiltered_control_signal_right;
  unfiltered_control_signal_right = kp * error + ki * integral + kd * derivative;

  //control_signal_right = control_signal_right_prev * 0.3891 + 0.3055 * (unfiltered_control_signal_right + unfiltered_control_signal_right_prev);
  control_signal_right = control_signal_right_prev * 0.4524 + 0.2738 * (unfiltered_control_signal_right + unfiltered_control_signal_right_prev);
  int pwm;
  if (fabs(control_signal_right) > 240)
    pwm = 240;
  else
    pwm = fabs(control_signal_right);
  

  analogWrite(MOTOR_RIGHT_PWM, pwm);
  digitalWrite(MOTOR_RIGHT_DIR, control_signal_right > 0);
}
*/