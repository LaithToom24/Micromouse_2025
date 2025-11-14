#include "functions.h"



long int current_time = 0;
long int last_time = 0;
int sampling_time = 10000;

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
  init_sensors();

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

  // detectWalls();
  // if(!wall_front)
  // {
  //   LeftWallPID(3.5f, 70.0f);
  // }
  // else
  // {
  //   turn(-90);
  // }
  // readAll();
  //LeftWallPID(1, 999.0);
  turn(90);




  // RightWallFollower();
  // delay(3000);
  // detectWalls();
  // readAll();
  // Serial.print("WF: ");
  // // Serial.print(wall_front);
  // Serial.print(front_dist);
  // Serial.print(" WR: ");
  // // Serial.print(wall_right);
  // Serial.print(right_dist);
  // Serial.print(" WL: ");
  // // Serial.print(wall_left);
  // Serial.print(left_dist);
  // Serial.println();





}
