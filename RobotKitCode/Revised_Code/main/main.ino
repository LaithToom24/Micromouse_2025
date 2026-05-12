#include "motor.hpp"
#include "solver.hpp"
#include "commands.hpp"
#include "API.hpp"
#include "sensor.hpp"
#include "gyroscopeSetup.hpp"

bool performing_command = false;

unsigned long solve_period = 10000;
unsigned long sensor_period = 5000; // update sensor readings every 5 ms
int control_period = 5000; // update control system every 5000 us = 5 ms
int vel_rate = 5; // update velocity every five control loops
int pos_rate = 1;

// velocity controller settings
float kf_v = 0.65f/1.5f;
float kp_v = 10.0f;
float ki_v = 3.33f;
float kd_v = 0.75f;

// rotational (wheel position) controller settings
/*
float kp_p = 1e-5f;
float ki_p = 50e-3f;
float kd_p = 10e-5f;
*/
float kp_p = 30e-3f;
float ki_p = 5.0e-3f;
float kd_p = 1e-3f;

// creating motor objects
Motor left_motor;
Motor right_motor;
PID_Controller turning_controller(kp_p, ki_p, kd_p, control_period, pos_rate, 1.0f, 7.0f);
PID_Controller drift_controller(1.0f, 0.35f, 0.1f, control_period, 1, 10.0f, 0.5f);

float position = 0;
float baseline_position = 0;

void setup() {
  Serial.begin(115200);

  // initialization of emitter pin
  pinMode(12, OUTPUT);

  // left motor initialization
  left_motor.set_motor_pins(9, 7, 2, 4);
  left_motor.set_interrupt(left_isr_CLK);
  left_motor.set_motor_specs(28, 39.0f, 3.0f, 7.0f);
  left_motor.set_velocity_controls(kf_v, kp_v, ki_v, kd_v, control_period, vel_rate, 3.0f);
  left_motor.set_motor_orientation(true);

  // right motor initialization
  right_motor.set_motor_pins(10, 8, 3, 5);
  right_motor.set_interrupt(right_isr_CLK);
  right_motor.set_motor_specs(28, 39.0f, 3.0f, 7.0f);
  right_motor.set_velocity_controls(kf_v, kp_v, ki_v, kd_v, control_period, vel_rate, 3.0f);
  right_motor.set_motor_orientation(false);

  gyro_init();
}

void loop(){
  //ramp_test();
  update_position(micros());
  if (micros() > 5e6){
      sensor_loop();
      solver_loop();
      //print_all();
  }
}

void solver_loop() {
  unsigned long now = micros();
  static unsigned long last_solve_time = micros();
  if (now - last_solve_time > solve_period){
  if (!performing_command){
    //print_commands();
    Action nextMove = solver();
    switch(nextMove){
        case FORWARD:
            //Serial.println("FORWARD");
            API_moveForward();
            performing_command = true;
            break;
        case LEFT:
            //Serial.println("LEFT");
            API_turnLeft();
            performing_command = true;
            break;
        case RIGHT:
            //Serial.println("RIGHT");
            API_turnRight();
            performing_command = true;
            break;
        case IDLE:
            break;
        default:
          break;
    }
  }
  last_solve_time = now;
  }
  command_loop();
}

void command_loop() {
  // put your main code here, to run repeatedly: 
  bool done = false;

  if (command_queue_size > 0){
    if (command_queue[0].type == 0)
      done = straight(command_queue[0].value); 
    else
      done = turn(command_queue[0].value);

    if (done){
      if (command_queue[0].type == 1)
        baseline_position += command_queue[0].value;
      remove_command();
      performing_command = false;
      right_motor.set_vel(0.0f);
      left_motor.set_vel(0.0f);
      //delayMicroseconds(1000)
      delay(1000);
      Serial.println("DONE");
    }
  }
  else{
    right_motor.set_vel(0.0f);
    left_motor.set_vel(0.0f);
  }
}

void sensor_loop(){
  unsigned long now = micros();
  static unsigned long last_sensor_time = micros();
  if (now - last_sensor_time > sensor_period){
    readAll();
    last_sensor_time = micros();
    //printAll();
  }
}

void update_position(unsigned long now){
  static unsigned long last_time = now;

  if (now - last_time >= 100){
    float dt = 1e-6f * (now - last_time);
    last_time = now;

    float speed = get_gyroZ();
    if (fabs(speed) > 0.5){
      position += 0.9935f*speed*dt;
      //Serial.println(position);
    }
  }
}

// robot commands
bool turn(float pos){
  static bool first_iteration = true;
  bool completed_turn = false;
  unsigned long now = micros();
  static unsigned long last_time = now;
  float vel = 0;
  static float target = 0;

  if (first_iteration){
    target = position + pos;
    last_time = now;
    first_iteration = false;
  }
  /*
  Serial.print(speed);
  Serial.print("\t");
  Serial.println(position);
  */
  float error = target - position;

  if (fabs(error) > 0.5f){
    vel = turning_controller.process(error, position, now);

    float min_turn_speed = 3.0f; // Adjust this (2.0 to 4.0)

    if (fabs(error) > 15.0f) { 
        if (vel > 0 && vel < min_turn_speed) vel = min_turn_speed;
        if (vel < 0 && vel > -min_turn_speed) vel = -min_turn_speed;
    }

    completed_turn = false;
  }
  else{
    turning_controller.process(now);
    turning_controller.clear();
    right_motor.reset_velocity();
    left_motor.reset_velocity();
    right_motor.reset_velocity_controller();
    left_motor.reset_velocity_controller();
    vel = 0;
    completed_turn = true;
    first_iteration = true;
  }

  vel = constrain(vel, -7.0f, 7.0f);

  left_motor.set_vel(vel);
  right_motor.set_vel(-vel);

  return completed_turn;
}

/*
bool straight(float distance){
  unsigned long now = micros();
  static bool first_iteration = true;
  static unsigned long last_time = now;
  static float distance_traveled = 0.0f;
  float correction = 0.0f;

  if (first_iteration){
    last_time = now;
    first_iteration = false;
  } 

  if (fabs(baseline_position - position) > 0.25f){
    correction = drift_controller.process(baseline_position - position, position, now);
  }

  right_motor.set_vel(5.0f - correction);
  left_motor.set_vel(5.0f + correction);

  distance_traveled += 5.0f*1e-6f*(now - last_time);
  last_time = now;
  if (distance_traveled >= distance){
    distance_traveled = 0.0f;
    first_iteration = true;
    right_motor.reset_velocity();
    left_motor.reset_velocity();
    right_motor.reset_velocity_controller();
    left_motor.reset_velocity_controller();
    return true;
  }
  return false;
}
*/

bool straight(float distance){
  static bool first_iteration = true;
  float correction = 0.0f;

  if (first_iteration){
    // Zero out the wheel odometry at the start of the straightaway
    left_motor.reset_position();
    right_motor.reset_position();
    first_iteration = false;
  } 

  // 1. Calculate gyro drift correction (unchanged)
  unsigned long now = micros();
  if (fabs(baseline_position - position) > 0.25f){
    correction = drift_controller.process(baseline_position - position, position, now);
  }

  // 2. Command the motors (unchanged)
  right_motor.set_vel(5.0f - correction);
  left_motor.set_vel(5.0f + correction);

  // 3. NEW: Calculate physical distance traveled using wheel encoders
  float left_traveled = left_motor.get_distance();
  float right_traveled = right_motor.get_distance();
  
  // Average the two wheels to find the true center-of-robot distance
  float distance_traveled = (left_traveled + right_traveled) / 2.0f;

  // 4. Check if we've reached the target distance
  if (fabs(distance_traveled) >= distance){
    first_iteration = true;
    right_motor.reset_velocity();
    left_motor.reset_velocity();
    right_motor.reset_velocity_controller();
    left_motor.reset_velocity_controller();
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



