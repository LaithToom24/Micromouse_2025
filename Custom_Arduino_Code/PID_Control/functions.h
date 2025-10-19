#include "variables.h"

void set_left_speed(float vel, float dtime){
  float error = vel - left_velocity;

  float integral = net_left_error + (error * dtime/1000.0f);
  float derivative = 1000.0f * (error - last_left_error) / dtime;

  net_left_error = integral;
  last_left_error = error;

  control_signal_left_prev = control_signal_left;
  unfiltered_control_signal_left_prev = unfiltered_control_signal_left;
  unfiltered_control_signal_left = kp * error + ki * integral + kd * derivative;

  //control_signal_left = control_signal_right_prev * 0.3891 + 0.3055 * (unfiltered_control_signal_right + unfiltered_control_signal_right_prev);
  control_signal_left = control_signal_left_prev * 0.4524 + 0.2738 * (unfiltered_control_signal_left + unfiltered_control_signal_left_prev);
  int pwm;
  if (fabs(control_signal_left) > 240)
    pwm = 240;
  else
    pwm = fabs(control_signal_left);
  

  analogWrite(MOTOR_LEFT_PWM, pwm);
  digitalWrite(MOTOR_LEFT_DIR, control_signal_left < 0);
}

void set_right_speed(float vel, float dtime){
  float error = vel - right_velocity;

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

void compute_right_speed(int right_count, int dtime){
  right_velocity_prev = right_velocity;
  unfiltered_right_vel_prev =  unfiltered_right_vel; 
  total_rotations_right = right_count / ((float)counts_per_rev * gear_ratio);
  if (digitalRead(MOTOR_RIGHT_DIR))
    unfiltered_right_vel = fabs(1000.0f * total_rotations_right * wheel_circumference / dtime);
  else
    unfiltered_right_vel = -fabs(1000.0f * total_rotations_right * wheel_circumference / dtime);

  right_velocity = -0.222 * right_velocity_prev + 0.6109 * (unfiltered_right_vel + unfiltered_right_vel_prev);

  if (digitalRead(MOTOR_RIGHT_DIR))
    right_velocity = fabs(right_velocity);
  else
    right_velocity = -fabs(right_velocity);
}

void compute_left_speed(int left_count, int dtime){
  left_velocity_prev = left_velocity;
  unfiltered_left_vel_prev =  unfiltered_left_vel; 
  total_rotations_left = left_count / ((float)counts_per_rev * gear_ratio);
  if (digitalRead(MOTOR_LEFT_DIR))
    unfiltered_left_vel = -fabs(1000.0f * total_rotations_left * wheel_circumference / dtime);
  else
    unfiltered_left_vel = fabs(1000.0f * total_rotations_left * wheel_circumference / dtime);

  left_velocity = -0.222 * left_velocity_prev + 0.6109 * (unfiltered_left_vel + unfiltered_left_vel_prev);

  if (digitalRead(MOTOR_LEFT_DIR))
    left_velocity = -fabs(left_velocity);
  else
    left_velocity = fabs(left_velocity);
}

void print_left_speed(){
  Serial.print("Left speed: ");
  Serial.print(left_velocity, 4);
  Serial.println(" cm/s");
}

void print_right_speed(){
  Serial.print("Right speed: ");
  Serial.print(right_velocity, 4);
  Serial.println(" cm/s");
}

void print_speeds(unsigned long t) {
  Serial.print(t);
  Serial.print(",");
  Serial.print(left_velocity, 4);
  Serial.print(",");
  Serial.println(right_velocity, 4);
}

// ISR Functions
void readLeftEncoderA(){
  int phaseA = digitalRead(ENCODER_LEFT_A);
  int phaseB = digitalRead(ENCODER_LEFT_B);
  //int phaseA = bitRead(PIND, 2);
  //int phaseB = bitRead(PIND, 4);

  //int current_pair = (phaseA << 1) + phaseB;
  //int past_pair = (left_phaseA << 1) + left_phaseB;
  //int sequence = (past_pair << 2) + current_pair;

  int encoded = (phaseA << 1) | phaseB;
  int sequence = (left_phaseA << 2) | encoded;

  if (sequence == 0b0001 || sequence == 0b0111 || sequence == 0b1110 || sequence == 0b1000){
    left_count++;
  }
  else if (sequence == 0b0010 || sequence == 0b1011 || sequence == 0b1101 || sequence == 0b0100){
    left_count--;
  }

  left_phaseA = phaseA;
  left_phaseB = phaseB;
}

void readLeftEncoderB(){
  int phaseA = digitalRead(ENCODER_LEFT_A);
  int phaseB = digitalRead(ENCODER_LEFT_B);
  //int phaseA = bitRead(PIND, 2);
  //int phaseB = bitRead(PIND, 4);

  //int current_pair = (phaseA << 1) + phaseB;
  //int past_pair = (left_phaseA << 1) + left_phaseB;
  //int sequence = (past_pair << 2) + current_pair;

  int encoded = (phaseA << 1) | phaseB;
  int sequence = (left_phaseA << 2) | encoded;

  if (sequence == 0b0001 || sequence == 0b0111 || sequence == 0b1110 || sequence == 0b1000){
    left_count++;
  }
  else if (sequence == 0b0010 || sequence == 0b1011 || sequence == 0b1101 || sequence == 0b0100){
    left_count--;
  }

  left_phaseA = phaseA;
  left_phaseB = phaseB;
}

void readRightEncoderA(){
  int phaseA = digitalRead(ENCODER_RIGHT_A);
  int phaseB = digitalRead(ENCODER_RIGHT_B);

  int encoded = (phaseA << 1) | phaseB;
  int sequence = (right_phaseA << 2) | encoded;
  //int current_pair = (phaseA << 1) + phaseB;
  //int past_pair = (right_phaseA << 1) + right_phaseB;
  //int sequence = (past_pair << 2) + current_pair;

  if (sequence == 0b0001 || sequence == 0b0111 || sequence == 0b1110 || sequence == 0b1000)
    right_count--;
  else if (sequence == 0b0010 || sequence == 0b1011 || sequence == 0b1101 || sequence == 0b0100)
    right_count++;

  right_phaseA = phaseA;
  right_phaseB = phaseB;
}

void readRightEncoderB(){
  int phaseA = digitalRead(ENCODER_RIGHT_A);
  int phaseB = digitalRead(ENCODER_RIGHT_B);

  int encoded = (phaseA << 1) | phaseB;
  int sequence = (right_phaseA << 2) | encoded;
  //int current_pair = (phaseA << 1) + phaseB;
  //int past_pair = (right_phaseA << 1) + right_phaseB;
  //int sequence = (past_pair << 2) + current_pair;

  if (sequence == 0b0001 || sequence == 0b0111 || sequence == 0b1110 || sequence == 0b1000)
    right_count--;
  else if (sequence == 0b0010 || sequence == 0b1011 || sequence == 0b1101 || sequence == 0b0100)
    right_count++;

  right_phaseA = phaseA;
  right_phaseB = phaseB;
}