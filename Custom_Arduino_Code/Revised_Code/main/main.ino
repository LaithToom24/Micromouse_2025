#include "motor.hpp"
#include "commands.hpp"

int control_period = 10000; // update control system every 10000 us = 10 ms
int vel_rate = 5; // update velocity every five control loops
int pos_rate = 1;

// velocity controller settings
float kf_v = 1.5f;
float kp_v = 40.5f;
float ki_v = 20.0f;
float kd_v = 0.25f;

// rotational (wheel position) controller settings
float kp_p = 5e-2f;
float ki_p = 1e-3f;
float kd_p = 5e-2f;

// creating motor objects
Motor left_motor;
Motor right_motor;
PID_Controller turning_controller(kp_p, ki_p, kd_p, control_period, pos_rate, 1.0f, 7.0f);

void setup() {
  Serial.begin(115200);

  // left motor initialization
  left_motor.set_motor_pins(9, 7, 2, 4);
  left_motor.set_interrupt(left_isr_CLK);
  left_motor.set_motor_specs(7, 39.0f, 3.0f, 7.0f);
  left_motor.set_velocity_controls(kf_v, kp_v, ki_v, kd_v, control_period, vel_rate, 3.0f);
  left_motor.set_motor_orientation(true);

  // right motor initialization
  right_motor.set_motor_pins(10, 8, 3, 5);
  right_motor.set_interrupt(right_isr_CLK);
  right_motor.set_motor_specs(7, 39.0f, 3.0f, 7.0f);
  right_motor.set_velocity_controls(kf_v, kp_v, ki_v, kd_v, control_period, vel_rate, 3.0f);
  right_motor.set_motor_orientation(false);

  /*
  // command robot to go in a square
  add_command(1, 90); // first command
  add_command(0, 6);
  add_command(1, 90);
  add_command(0, 6);
  add_command(1, 90);
  add_command(0, 6);
  add_command(1, 90);
  add_command(0, 6);  // last command
  */

  // command robot to go in a triangle
  add_command(1, 60); // first command
  add_command(0, 6);
  add_command(1, 60);
  add_command(0, 6);
  add_command(1, 160);
  add_command(0, 12);  // last command
}

void loop() {
  // put your main code here, to run repeatedly: 
  bool done = false;

  if (queue_size > 0){
    if (command_queue[0].type == 0)
      done = straight(command_queue[0].value); 
    else
      done = turn(command_queue[0].value);

    if (done)
      remove_command();
  }
  else{
    right_motor.set_vel(0.0f);
    left_motor.set_vel(0.0f);
  }
}

// robot commands
bool turn(float pos){
  static bool first_iteration = true;
  bool completed_turn = false;
  unsigned long now = micros();
  float vel = 0;

  if (first_iteration){
    left_motor.reset_position();
    right_motor.reset_position();
    first_iteration = false;
  }

  pos = 2.5*constrain(pos, -360.0f, 360.0f);

  float position = 0.5f*(left_motor.get_pos() + fabs(right_motor.get_pos()));
  float error = pos - position;

  if (fabs(error) > 10.0f){
    vel += turning_controller.process(error, position, now);
    completed_turn = false;
  }
  else{
    turning_controller.process(now);
    turning_controller.clear();
    vel = 0;
    completed_turn = true;
    first_iteration = true;
  }

  vel = constrain(vel, -7.0f, 7.0f);

  left_motor.set_vel(vel);
  right_motor.set_vel(-vel);

  if (turning_controller.get_control_loops() == pos_rate){
    left_motor.update_pos();
    right_motor.update_pos();
  }

  return completed_turn;
}

bool straight(float distance){
  unsigned long now = micros();
  static bool first_iteration = true;
  static unsigned long last_time = now;
  static float distance_traveled = 0.0f;

  if (first_iteration){
    last_time = now;
    first_iteration = false;
  } 

  right_motor.set_vel(5.0f);
  left_motor.set_vel(5.0f);

  distance_traveled += 5.0f*1e-6f*(now - last_time);
  last_time = now;
  if (distance_traveled >= distance){
    distance_traveled = 0.0f;
    first_iteration = true;
    return true;
  }
  return false;
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



