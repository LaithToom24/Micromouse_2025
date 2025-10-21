#include "functions.h"

void setup() {
  Serial.begin(115200);

  init_motor(&left_motor, 9, 7, 2, 4, 7, 39.0, true);
  init_motor(&right_motor, 10, 8, 3, 5, 7, 39.0, false);
  set_PID_coeffs(&left_motor, 30, 50, 2);
  set_PID_coeffs(&right_motor, 30, 50, 2);

  attachInterrupt(digitalPinToInterrupt(left_motor.ENCODER_AxorB), readLeftEncoderA, RISING);
  //attachInterrupt(digitalPinToInterrupt(left_motor.ENCODER_B), readLeftEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_motor.ENCODER_AxorB), readRightEncoderA, RISING);
  //attachInterrupt(digitalPinToInterrupt(right_motor.ENCODER_B), readRightEncoderB, CHANGE);

  last_time_ctrl = micros();
  last_time_vel = last_time_ctrl;
}

void loop() {
  current_time = micros();
  if (current_time <= 10e6){
    if (current_time - last_time_ctrl >= sampling_time){
      compute_controls(-5, -5);
      control_loops++;
      //print_controls(micros());
      last_time_ctrl = current_time;
    }

    if (control_loops == velocity_window){
      control_loops = 0;
      compute_speeds();
      print_speeds(micros());
      last_time_vel = current_time;
    }
  }
  else{
    if (current_time >= 15e6)
      Serial.end();
    if (current_time - last_time_ctrl >= sampling_time){
      compute_controls(0, 0);
      control_loops++;
      //if (current_time <= 15e6)
        //print_controls(micros());
      last_time_ctrl = current_time;
    }

    if (control_loops == velocity_window){
      control_loops = 0;
      compute_speeds();
      if (current_time <= 15e6)
        print_speeds(micros());
      last_time_vel = current_time;
    }
  }

  set_speeds();
}
