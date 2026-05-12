#include "motor.hpp"
#include "solver.hpp"
#include "commands.hpp"
#include "API.hpp"
#include "sensor.hpp"
#include "gyroscope.hpp"

bool performing_command = false;
bool wallLeft = false;
bool wallRight = false;
bool wallFront = false;
int leftDistance = 0;
int rightDistance = 0;
int frontDistance = 0;
unsigned long solve_period = 10000;
unsigned long sensor_period = 120; // update sensor readings every 5 ms
int control_period = 5000; // update control system every 5000 us = 5 ms
int vel_rate = 1; // update velocity every five control loops
int pos_rate = 1;

// set nominal speed
float nominalSpeed = 1.425f;
// set front detection distance
int frontDistanceThreshold = 45;

// velocity controller settings
float kf_v = 1.0f;
float kp_v = 0.1f;
float ki_v = 0.01f;
float kd_v = 1e-4f;

// rotational (wheel position) controller settings
/*
float kp_p = 1e-5f;
float ki_p = 50e-3f;
float kd_p = 10e-5f;
*/
float kp_p = 4.5e-3f;
//float ki_p = 1e-4f;
float ki_p = 2.75e-3f;
float kd_p = 1.25e-3f;

//float kp_p = 1e-4f;
//float ki_p = 1e-2f;
//float kd_p = 5e-4f;
// creating motor objects
Motor left_motor;
Motor right_motor;
//PID_Controller turning_controller(kp_p, ki_p, kd_p, control_period, pos_rate, 1.0f, 2.0f);
PID_Controller turning_controller(kp_p, ki_p, kd_p, control_period, pos_rate, 1.0f, 0.95f);
PID_Controller drift_controller(5e-3f, 5e-3f, 1e-7f, control_period, 1, 9.0f, 0.325f);

float position = 0;
float baseline_position = 0;

void setup() {
  Serial.begin(115200);

  // Enabling On-Board LED Indicators
  pinMode(A0, OUTPUT);
  pinMode(A6, OUTPUT);
  pinMode(A7, OUTPUT);

  // Light Up LEDs
  // Left LED
  digitalWrite(A7, LOW);
  digitalWrite(A6, LOW);
  digitalWrite(A0, HIGH);
  delay(250);
  // Front LED
  digitalWrite(A7, HIGH);
  digitalWrite(A6, LOW);
  digitalWrite(A0, HIGH);
  delay(250);
  // Right LED
  digitalWrite(A7, HIGH);
  digitalWrite(A6, HIGH);
  digitalWrite(A0, HIGH);
  delay(250);
  digitalWrite(A7, LOW);
  digitalWrite(A6, LOW);
  digitalWrite(A0, LOW);
  delay(250);
  // Enable the motor driver by pulling STBY high
  pinMode(D8, OUTPUT);
  digitalWrite(D8, HIGH);
  // Ri motor initialization (Motor 1 on PCB)
  // Pins: PWM=11, IN1=9, IN2=10, EncA=12, EncB=13
  left_motor.set_motor_pins(D11, D9, D10, D12, D13);
  left_motor.set_interrupt(left_isr_CLK);
  left_motor.set_motor_specs(12, 15.0f, 3.0f, 7.0f);
  left_motor.set_velocity_controls(kf_v, kp_v, ki_v, kd_v, control_period, vel_rate, 2.0f);
  left_motor.set_motor_orientation(true);
  // Right motor initialization (Motor 2 on PCB)
  // Pins: PWM=5, IN1=6, IN2=7, EncA=2, EncB=3
  right_motor.set_motor_pins(D5, D6, D7, D2, D3);
  right_motor.set_interrupt(right_isr_CLK);
  right_motor.set_motor_specs(12, 15.0f, 3.0f, 7.0f);
  right_motor.set_velocity_controls(kf_v, kp_v, ki_v, kd_v, control_period, vel_rate, 2.0f);
  right_motor.set_motor_orientation(false);

  gyro_init();
  ToF_setup();
  // Indicator LEDs
  /*
  // Left LED
  digitalWrite(A7, LOW);
  digitalWrite(A6, LOW);
  digitalWrite(A0, HIGH);
  delay(2000);
  // Front LED
  digitalWrite(A7, LOW);
  digitalWrite(A6, HIGH);
  digitalWrite(A0, LOW);
  delay(2000);
  // Right LED
  digitalWrite(A7, HIGH);
  digitalWrite(A6, LOW);
  digitalWrite(A0, LOW);
  delay(2000);
  digitalWrite(A7, LOW);
  digitalWrite(A6, LOW);
  digitalWrite(A0, LOW);
  */

  // setup switch
  pinMode(D4, INPUT);
  // Give the microcontroller a moment to relax
  delay(100);
  if (digitalRead(D4)){
    // test functions for sensors
    ToF_test();
    gyro_test();
  }
  else{
    Serial.println("Bypassing Tests");
  }

  //add_command(0, 1.75);
  //add_command(1, 90);
  //add_command(0, 10);
  //add_command(1, 90);
}

void loop(){
  //ramp_test();
  unsigned long time_now = micros();
  update_position(time_now);
  sensor_loop();
  if (time_now >= 5e6){
    if (performing_command == false && command_queue_size == 0) {
      // The robot has finished moving and the queue is empty.
      // NOW it is safe to look at the sensors, update the maze array, 
      // run Floodfill, and push the next command(s) into the queue.
      solver_loop(); 
    }
    command_loop();
  }
  
  /*
  Serial.print("Left: ");
  Serial.print(leftDistance);
  Serial.print("\tRight: ");
  Serial.print(rightDistance);
  Serial.print("\tFront: ");
  Serial.println(frontDistance);
  */
  
}

enum WanderState {
  DRIVE_STRAIGHT,
  TURN_IN_PLACE
};

void solver_loop() {
  unsigned long now = micros();
  static unsigned long last_solve_time = micros();
  if (now - last_solve_time > solve_period){
  if (!performing_command){
    //print_commands();
    Action nextMove = solver(wallFront, wallLeft, wallRight);
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
  static unsigned long wait_start_time = 0;
  static bool is_waiting = false;
  // Define the non-blocking wait duration: 150ms to allow physical settling and ToF updates
  const unsigned long SETTLE_DELAY_US = 33000; 

  // 1. NON-BLOCKING WAIT STATE
  // If the robot recently finished a command, it enters this block.
  if (is_waiting) {
    if (micros() - wait_start_time >= SETTLE_DELAY_US) {
      // The wait duration has elapsed.
      is_waiting = false;
      performing_command = false; // Unlock the solver so it can read the fresh sensor data
    } else {
      // Still waiting. Keep motors stopped and ensure the solver remains locked.
      right_motor.set_vel(0.0f);
      left_motor.set_vel(0.0f);
      performing_command = true; 
      return; // Exit the function early; do not process new commands
    }
  }

  // 2. STANDARD EXECUTION STATE
  bool done = false;
  if (command_queue_size > 0){
    if (command_queue[0].type == 0)
      done = straight(command_queue[0].value);
    else
      done = turn(command_queue[0].value);

    // Lock the solver while a command is actively executing
    performing_command = true;
    if (done){
      if (command_queue[0].type == 1)
        baseline_position += command_queue[0].value;
      remove_command();
      right_motor.set_vel(0.0f);
      left_motor.set_vel(0.0f);
      
      // --- TRIGGER THE NON-BLOCKING WAIT ---
      // The command has finished. Instead of halting the processor, 
      // transition to the wait state to allow sensors to settle.
      is_waiting = true;
      wait_start_time = micros();
      
      Serial.println("Command complete. Entering sensor settle phase.");
    }
  }
  else{
    right_motor.set_vel(0.0f);
    left_motor.set_vel(0.0f);
    // Ensure the solver is unlocked if the queue is empty and no wait is pending
    if (!is_waiting) {
      performing_command = false;
    }
  }
}

void sensor_loop(){
  unsigned long now = micros();
  static unsigned long last_sensor_time = micros();
  
  // Create confidence counters
  static int front_confidence = 0;
  static int left_confidence = 0;
  static int right_confidence = 0;
  if (now - last_sensor_time > sensor_period){
    last_sensor_time = micros();
    // Grab the new readings
    leftDistance = readLeft();
    rightDistance = readRight();
    frontDistance = readFront();
    
    // --- CONFIDENCE FILTERING ---
    // FRONT WALL
    if (frontDistance <= 100){
      front_confidence++;
    } else {
      front_confidence = 0; // Reset immediately if the path is clear
    }
    // Only declare a wall if we saw it 2 times in a row!
    wallFront = (front_confidence >= 2); 

    // RIGHT WALL
    if (rightDistance <= 85) {
      right_confidence++;
    } else {
      right_confidence = 0;
    }
    wallRight = (right_confidence >= 2);
    
    // LEFT WALL
    if (leftDistance <= 85) {
      left_confidence++;
    } else {
      left_confidence = 0;
    }
    wallLeft = (left_confidence >= 2);
    
    // Update LEDs
    digitalWrite(A0, wallLeft);
    digitalWrite(A7, wallRight);
    digitalWrite(A6, wallFront);
  }
}

void update_position(unsigned long now){
  static unsigned long last_time = now;
  if (now - last_time >= 100){
    float dt = 1e-6f * (now - last_time);
    last_time = now;
    float speed = get_gyroZ();
    // Removed the deadband here to allow micro-adjustments
    position += 0.9935f*speed*dt;
  }
}

// Robot commands
bool turn(float pos){
  static bool first_iteration = true;
  bool completed_turn = false;
  unsigned long now = micros();
  float vel = 0;
  static float target = 0;
  
  if (first_iteration){
    target = position + pos;
    first_iteration = false;
  }

  float error = target - position;

  // Tighter exit tolerance (2.0 to 5.0 degrees)
  if (fabs(error) > 2.0f){
    vel = turning_controller.process(error, position, now);
    
    // Continuous Friction Feedforward
    if (vel < 0.0f){
      vel -= 0.53f;
    }
    else if (vel > 0.0f){
      vel += 0.53f;
    }

    // Minimum Velocity Floor to prevent lockups near the target
    if (vel > 0.0f && vel < 0.75f) { 
        vel = 0.75f;
    } else if (vel < 0.0f && vel > -0.75f) {
        vel = -0.75f;
    }

    // Safety Clamp
    //vel = constrain(vel, -7.0f, 7.0f);
    // Apply the effort (Left goes +, Right goes -)
    left_motor.set_vel(vel);
    right_motor.set_vel(-vel);

    completed_turn = false;
  }
  else{
    // The turn is complete!
    turning_controller.process(now);
    turning_controller.clear();
    
    // Clear the drift controller here based on your preference
    drift_controller.clear(); 
    
    right_motor.reset_velocity();
    left_motor.reset_velocity();
    right_motor.reset_velocity_controller();
    left_motor.reset_velocity_controller();
    // CRITICAL: Let the physical ToF hardware take a fresh picture of the new path!
    // This prevents the solver from using "stale" data from before the turn.
    // delay(50); 
    
    vel = 0;
    completed_turn = true;
    first_iteration = true;
  }

  return completed_turn;
}

bool straight(float distance){
  static bool first_iteration = true;
  float correction = 0.0f;
  float wallCorrect = 0.0f;

  if (first_iteration){
    // Zero out the wheel odometry at the start of the straightaway
    left_motor.reset_position();
    right_motor.reset_position();
    first_iteration = false;
  } 

  unsigned long now = micros();
  // --- Emergency Abort ---
  // If we are physically crashing, the wheels will stall and the encoders
  // will never reach the target distance. We must abort optically!
  if (frontDistance < 45 && frontDistance > 0) { 
    first_iteration = true;
    right_motor.reset_velocity();
    left_motor.reset_velocity();
    right_motor.reset_velocity_controller();
    left_motor.reset_velocity_controller();
    return true; // Bail out and let the solver take over!
  }
  // -----------------------------------------

  // 1. Calculate gyro drift correction
  if (fabs(baseline_position - position) > 0.1f){
    correction = drift_controller.process(baseline_position - position, position, now);
  }

  // 2. Calculate Proportional Wall Avoidance (using your current tuning)
  if (rightDistance <= 35) {
    wallCorrect = -0.275f * (1.0f - ((float)rightDistance / 125.0f));
  }
  else if (leftDistance <= 35) {
    wallCorrect = 0.275f * (1.0f - ((float)leftDistance / 125.0f));
  }

  // 3. Command the motors differentially (No Braking)
  if (frontDistance <= 250){
    right_motor.set_vel((0.4f + (float)frontDistance/250.0f) * nominalSpeed - correction - wallCorrect);
    left_motor.set_vel((0.4f + (float)frontDistance/250.0f) * nominalSpeed + correction + wallCorrect);
  }
  else{
    right_motor.set_vel(nominalSpeed - correction - wallCorrect);
    left_motor.set_vel(nominalSpeed + correction + wallCorrect);
  }

  // 4. Calculate physical distance traveled using wheel encoders
  float left_traveled = left_motor.get_distance();
  float right_traveled = right_motor.get_distance();
  float distance_traveled = (left_traveled + right_traveled) / 2.0f;
  
  // 5. Check if we've reached the target distance normally (ignoring out-of-range zero readings)
  if (fabs(distance_traveled) >= distance || (frontDistance < frontDistanceThreshold && frontDistance > 0)){
    first_iteration = true;
    right_motor.reset_velocity();
    left_motor.reset_velocity();
    right_motor.reset_velocity_controller();
    left_motor.reset_velocity_controller();
    //drift_controller.clear(); 
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