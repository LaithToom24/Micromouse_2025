#include "functions.h"

void setup() {
  Serial.begin(115200);

  init_motor(&left_motor, 9, 7, 2, 4, 7, 39.0, true);
  init_motor(&right_motor, 10, 8, 3, 5, 7, 39.0, false);
  set_PID_coeffs(&left_motor, 1, 0, 1);
  set_PID_coeffs(&right_motor, 1, 0, 1);

  attachInterrupt(digitalPinToInterrupt(left_motor.ENCODER_A), readLeftEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(left_motor.ENCODER_B), readLeftEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_motor.ENCODER_A), readRightEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(right_motor.ENCODER_B), readRightEncoderB, CHANGE);

  last_time_ctrl = micros();
  last_time_vel = last_time_ctrl;
}

void loop() {
  current_time = micros();
  if (current_time <= 10*1000000){
    if (current_time - last_time_ctrl >= sampling_time){
      compute_controls(5, 5);
      //print_controls(current_time);
      last_time_ctrl = current_time;
    }

    if (current_time - last_time_vel >= velocity_window){
      compute_speeds();
      print_speeds(current_time);
      last_time_vel = current_time;
    }
  }
  else{
    if (current_time >= 15*1000000)
      Serial.end();
    if (current_time - last_time_ctrl >= sampling_time){
      compute_controls(0, 0);
      //print_controls(current_time);
      last_time_ctrl = current_time;
    }

    if (current_time - last_time_vel >= velocity_window){
      compute_speeds();
      if (current_time <= 15*1000000)
        print_speeds(current_time);
      last_time_vel = current_time;
    }
  }

  set_speeds();
}