#include "motor.h"

int control_period = 10000; // update control system every 5000 us = 5 ms
int velocity_period = 4; // update velocity every four control loops
int position_period = 4;

Motor left_motor(9, 7, 2, 4, 7, 39.0f, 3.0f, true, 2.60f, 19.0f, 21.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.0f, control_period, velocity_period, position_period, 2.0f, 7.0f);
Motor right_motor(10, 8, 3, 5, 7, 39.0f, 3.0f, false, 2.60f, 19.0f, 21.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.0f, control_period, velocity_period, position_period, 2.0f, 7.0f);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  left_motor.init(left_isr);
  right_motor.init(right_isr);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  unsigned long time = micros();  
  static unsigned long last_time = 0;
  
  //ramp_test(time);

  //turn(90);

  opposite_speed(1.0f);

  if ((time - last_time > (float)position_period*control_period)){
      Serial.print(left_motor.get_pos(), 2);
      Serial.print(",");
      Serial.print(right_motor.get_pos(), 2);
      Serial.print(",");
      Serial.println(micros());
      last_time = time;
  }
  
}

// isr handlers for motors
void left_isr(){
  left_motor.readEncoder();
}
void right_isr(){
  right_motor.readEncoder();
}

// robot functions

void step_test(unsigned long time){
  static unsigned long last_time = 0;

  float vel = 0.0f;

  if (time < 10e6)
    vel = 3.0f;

  opposite_speed(vel);

  
  if ((time - last_time > control_period) && time < 15e6){
      Serial.print(left_motor.get_vel(), 2);
      Serial.print(",");
      Serial.print(right_motor.get_vel(), 2);
      Serial.print(",");
      Serial.print(vel);
      Serial.print(",");
      Serial.print(vel);
      Serial.print(",");
      Serial.print(left_motor.get_pwm(), 2);
      Serial.print(",");
      Serial.print(right_motor.get_pwm(), 2);
      Serial.print(",");
      Serial.println(micros());
      last_time = time;
  }
  
}

void ramp_test(unsigned long time){
  static unsigned long last_time = 0;
  static float vel;

  if (time < 10e6){
    if (vel < 5.0f)
      vel = time * (20.0f / 5e6f);
    else
      vel = 5.0f;
  }
  else
    vel = 0.0f;

  straight(vel);

  
  if ((time - last_time > control_period) && time < 15e6){
      Serial.print(left_motor.get_vel(), 2);
      Serial.print(",");
      Serial.print(right_motor.get_vel(), 2);
      Serial.print(",");
      Serial.print(vel, 2);
      Serial.print(",");
      Serial.print(vel, 2);
      Serial.print(",");
      Serial.print(left_motor.get_pwm(), 2);
      Serial.print(",");
      Serial.print(right_motor.get_pwm(), 2);
      Serial.print(",");
      Serial.println(micros());
      last_time = time;
  }
  
}


void turn(float pos){
  right_motor.set_pos(pos);
  left_motor.set_pos(-pos);
}


void straight(float vel){
  right_motor.set_vel(vel);
  left_motor.set_vel(vel);
}

void opposite_speed(float vel){
  right_motor.set_vel(vel);
  left_motor.set_vel(-vel);
}

