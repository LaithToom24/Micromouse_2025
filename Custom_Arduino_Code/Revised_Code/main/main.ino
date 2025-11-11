#include "motor.hpp"

int control_period = 10000; // update control system every 10000 us = 10 ms
int velocity_period = 4; // update velocity every four control loops
int position_period = 1;

// velocity controller settings
float kf_v = 1.9f;
float kp_v = 47.5f;
float ki_v = 31.0f;
float kd_v = 0.5f;

// rotational (wheel position) controller settings
float kf_p = 0.0f;
float kp_p = 1.0f;
float ki_p = 0.0f;
float kd_p = 0.0f;

//Motor left_motor(9, 7, 2, 4, 7, 39.0f, 3.0f, true, 2.60f, 19.0f, 21.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.0f, control_period, velocity_period, position_period, 2.0f, 7.0f);
//Motor right_motor(10, 8, 3, 5, 7, 39.0f, 3.0f, false, 2.60f, 19.0f, 21.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.0f, control_period, velocity_period, position_period, 2.0f, 7.0f);

Motor left_motor(9, 7, 2, 4, 7, 39.0f, 3.0f, true, kf_v, kp_v, ki_v, kd_v, kf_p, kp_p, ki_p, kd_p, control_period, velocity_period, position_period, 3.0f, 7.0f);
Motor right_motor(10, 8, 3, 5, 7, 39.0f, 3.0f, false, kf_v, kp_v, ki_v, kd_v, kf_p, kp_p, ki_p, kd_p, control_period, velocity_period, position_period, 3.0f, 7.0f);

void setup() {
  Serial.begin(115200);

  // Assign pins and interrupts
  left_motor.init(left_isr_CLK);
  right_motor.init(right_isr_CLK);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  unsigned long time = micros();  
  static unsigned long last_time = 0;
  
  turn(90);

  /*
  if ((time - last_time > (float)position_period*control_period)){
      Serial.print(left_motor.get_pos(), 2);
      Serial.print(",");
      Serial.print(right_motor.get_pos(), 2);
      Serial.print(",");
      Serial.println(micros());
      last_time = time;
  }
  */
  
}

// robot commands
void turn(float pos){
  right_motor.set_pos(pos*2);
  left_motor.set_pos(-pos*2);
}

void straight(float vel){
  right_motor.set_vel(vel);
  left_motor.set_vel(vel);
}

// ISRs

// isr handlers for left motor
void left_isr_CLK(){
  left_motor.readEncoder();
}

// isr handlers for right motor
void right_isr_CLK(){
  right_motor.readEncoder();
}

// robot tests
void step_test(unsigned long time){
  static unsigned long last_time = 0;

  float vel = 0.0f;

  if (time < 10e6)
    vel = 3.0f;

  opposite_speed(vel);

  
  if ((time - last_time > (float)velocity_period * control_period) && time < 15e6){
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

  straight(-vel);
  
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

void turn_test(unsigned long time){
  static unsigned long last_time = 0;

  turn(90);

  /*
  if ((time - last_time > (float)position_period*control_period)){
      Serial.print(left_motor.get_pos(), 2);
      Serial.print(",");
      Serial.print(right_motor.get_pos(), 2);
      Serial.print(",");
      Serial.println(micros());
      last_time = time;
  }
  */
}

void opposite_speed(float vel){
  right_motor.set_vel(vel);
  left_motor.set_vel(-vel);
}



