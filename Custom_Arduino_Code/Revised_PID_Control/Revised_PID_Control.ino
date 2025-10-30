#include "functions.h"

bool first_turn = false;
bool second_turn = false;

void setup() {
  Serial.begin(115200);

  static float Kp = 12.5f;
  static float Ki = 200.0f;
  static float Kd = 0.05f;
  static float cutoff_freq = 2.0f;

  init_motor(&left_motor, 9, 7, 2, 4, 7, 39.0, cutoff_freq, true);
  init_motor(&right_motor, 10, 8, 3, 5, 7, 39.0, cutoff_freq, false);
  set_PID_coeffs(&left_motor, Kp, Ki, Kd);
  set_PID_coeffs(&right_motor, Kp, Ki, Kd);

  attachInterrupt(digitalPinToInterrupt(left_motor.ENCODER_AxorB), readLeftEncoderA, RISING);
  //attachInterrupt(digitalPinToInterrupt(left_motor.ENCODER_B), readLeftEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_motor.ENCODER_AxorB), readRightEncoderA, RISING);
  //attachInterrupt(digitalPinToInterrupt(right_motor.ENCODER_B), readRightEncoderB, CHANGE);

  last_time = micros();

  Serial.print(Kp, 1);
  Serial.print(",");
  Serial.print(Ki, 1);
  Serial.print(",");
  Serial.print(Kd, 1);
  Serial.print(",");
  Serial.print(cutoff_freq, 1);
  Serial.print(",");
  Serial.print(sampling_time, 1);
  Serial.print(",");
  Serial.print(sampling_time*velocity_window, 1);
  Serial.println();
}

void loop() {
  current_time = micros();
  long elapsed_time = current_time - last_time;
  bool test_over = (elapsed_time >= 18e6);

  /*
  
  float left_target, right_target;

  if (current_time <= 3e6){
    left_target = 5.0f;
    right_target = 5.0f;
  }
  else if (current_time <= 6e6){
    left_target = 3.0f;
    right_target = 3.0f;
  }
  else if (current_time <= 9e6){
    left_target = 7.0f;
    right_target = 7.0f;
  }
  else if (current_time <= 12e6){
    left_target = -7.0f;
    right_target = -7.0f;
  }
  else if (current_time <= 15e6){
    left_target = 3.0f;
    right_target = 3.0f;
  }
  else{
    left_target = 0.0f;
    right_target = 0.0f;
  }

  if (elapsed_time >= sampling_time){
    compute_controls(-left_target, right_target);
    control_loops++;
    if (!test_over)
      print_speeds_and_controls(micros());
    last_time = current_time;
  }

  if (control_loops == velocity_window){
    control_loops = 0;
    compute_speeds();
  }

  set_speeds();
  */

  //Serial.print(first_turn);
  //Serial.print(",");
  //Serial.print(second_turn);
  //Serial.print(",");
  //Serial.println(done_turning);

  if (elapsed_time >= sampling_time){
    compute_controls(left_target, right_target);
    control_loops++;
    //if (!test_over)
      //print_speeds_and_controls(micros());
    last_time = current_time;
  }

  if (control_loops == velocity_window){
    control_loops = 0;
    compute_speeds();
  }

  set_speeds();
}
