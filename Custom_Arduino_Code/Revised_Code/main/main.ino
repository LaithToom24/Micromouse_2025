#include "motor.hpp"

int control_period = 10000; // update control system every 10000 us = 10 ms
int vel_rate = 4; // update velocity every four control loops
int pos_rate = 1;

// velocity controller settings
float kf_v = 1.9f;
float kp_v = 47.5f;
float ki_v = 31.0f;
float kd_v = 0.5f;

// rotational (wheel position) controller settings
float kf_p = 0.0f;
float kp_p = 2.0f;
float ki_p = 0.0f;
float kd_p = 0.0f;

// creating motor objects
Motor left_motor;
Motor right_motor;

bool turn_commanded = false;

void setup() {
  Serial.begin(115200);

  // left motor initialization
  left_motor.set_motor_pins(9, 7, 2, 4);
  left_motor.set_interrupt(left_isr_CLK);
  left_motor.set_motor_specs(7, 39.0f, 3.0f, 7.0f);
  left_motor.set_velocity_controls(kf_v, kp_v, ki_v, kd_v, control_period, vel_rate, 3.0f);
  left_motor.set_position_controls(kf_p, kp_p, ki_p, kd_p, control_period, pos_rate, 1.0f);
  left_motor.set_motor_orientation(true);

  // right motor initialization
  right_motor.set_motor_pins(10, 8, 3, 5);
  right_motor.set_interrupt(right_isr_CLK);
  right_motor.set_motor_specs(7, 39.0f, 3.0f, 7.0f);
  right_motor.set_velocity_controls(kf_v, kp_v, ki_v, kd_v, control_period, vel_rate, 3.0f);
  right_motor.set_position_controls(kf_p, kp_p, ki_p, kd_p, control_period, pos_rate, 1.0f);
  right_motor.set_motor_orientation(false);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long time = micros();  
  static unsigned long last_time = 0;
  
  turn(360*2);
  
  if ((time - last_time > (float)pos_rate*control_period)){
      Serial.print(left_motor.get_pos(), 2);
      Serial.print(",");
      Serial.print(right_motor.get_pos(), 2);
      Serial.print(",");
      Serial.println(micros());
      last_time = time;
  }
  
  /*
  if ((time - last_time > (float)vel_rate * control_period) && time < 15e6){
      Serial.print(left_motor.get_vel(), 2);
      Serial.print(",");
      Serial.print(right_motor.get_vel(), 2);
      Serial.print(",");
      Serial.println(micros());
      last_time = time;
  }
  */
  
}

// robot commands
void turn(float pos){
  static bool last_pos = 0;

  if (pos != last_pos){
    turn_commanded = true;
    //right_motor.reset_rotational_encoder_count();
    //left_motor.reset_rotational_encoder_count();
    last_pos = pos;
  }

  right_motor.set_pos(pos*3);
  left_motor.set_pos(-pos*2);

  if (!right_motor.is_turning() && !left_motor.is_turning())
    turn_commanded = false;
}

void straight(float vel){
  if (turn_commanded)
    return;

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

  
  if ((time - last_time > (float)vel_rate * control_period) && time < 15e6){
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
  if ((time - last_time > (float)pos_rate*control_period)){
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



