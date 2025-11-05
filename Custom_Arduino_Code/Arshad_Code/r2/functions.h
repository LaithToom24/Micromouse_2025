#include "declarations.h"
#include <digitalWriteFast.h>
#include <util/atomic.h>

// Individual motor functions

void init_motor(Motor* motor, unsigned short PWM, unsigned short DIR, unsigned short ENCODER_AxorB, unsigned short ENCODER_B, int specified_CPR, float gear_ratio, float cutoff_freq, bool LEFT){
  motor -> PWM = PWM;
  motor -> DIR = DIR;
  motor -> ENCODER_AxorB = ENCODER_AxorB;
  motor -> ENCODER_B = ENCODER_B;
  motor -> specified_CPR = specified_CPR;
  motor -> gear_ratio = gear_ratio;
  motor -> CPR = specified_CPR * gear_ratio;
  motor -> PPR = 4 * motor -> CPR;
  motor -> LEFT = LEFT;

  compute_lowpass_filter_coeffs(motor, sampling_time, cutoff_freq);

  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(ENCODER_AxorB, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
}


void set_PID_coeffs(Motor* motor, float Kp, float Ki, float Kd){
  motor -> Kp = Kp;
  motor -> Ki = Ki;
  motor -> Kd = Kd;
}

void compute_speed(Motor* motor){
  motor -> derivative[0] = motor -> derivative[1]; 
  motor -> RPM[0] = motor -> RPM[1];
  motor -> velocity[0] = motor -> velocity[1];
  motor -> filtered_velocity[0] = motor -> filtered_velocity[1];
  long int dcount;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    dcount = motor -> encoder_count;
    // update last encoder count
    motor -> encoder_count = 0;
  }

  float revolutions = ((float)dcount / motor -> PPR)/5.0f;
  float RPM = (60*1e6f) * revolutions / ((float)velocity_window * (float)sampling_time);
  float speed = fabs( (wheel_circumference * 1e6f * (float)dcount) / (motor -> PPR * (float)velocity_window * (float)sampling_time) )/5.0f;
  float velocity;

  if (digitalReadFast(motor -> DIR) == motor -> LEFT)
    velocity = -speed;
  else
    velocity = speed;

  //motor->velocity[1] = (wheel_circumference * (float)dcount * 1e6f) / (motor->PPR * velocity_window);

  motor -> RPM[1] = RPM;
  motor -> velocity[1] = velocity;

  // low-pass filter for velocity measurement
  //motor -> filtered_velocity[1] = 0.5983 * motor -> filtered_velocity[0] + 0.2008 * (motor -> velocity[1] + motor -> velocity[0]);

  motor -> derivative[1] =  (2 / ((float)velocity_window * (float)sampling_time/1e6f)) * (motor -> velocity[1] - motor -> velocity[0]) - motor -> derivative[0]; 

  // low-pass filter for derivative term
  motor -> filtered_derivative[1] = motor -> b_d * motor -> filtered_derivative[0] + motor -> a_d * (motor -> derivative[1] + motor -> derivative[0]);
}

void compute_control(Motor* motor, float target){
  if (target > 7)
    target = 7;
  else if (target < -7)
    target = -7;
  else if (fabs(target) < 0.1){
    motor -> control[1] = 0;
    motor -> filtered_control[1] = 0;
    return;
  }

  /*

  motor -> error[0] = motor -> error[1];
  motor -> error[1] = target - motor -> filtered_velocity[1];

  motor -> integral[1] = motor -> integral[0] + 0.5 * ((float)sampling_time/1e6f) * (motor -> error[1] + motor -> error[0]);

  float integral_max = 240.0f / max(0.001f, motor -> Ki);
  // Check for integral saturation.
  motor -> integrator_saturated = fabs(motor -> integral[1]) >= integral_max;

  // If integral is not saturated, continue. Else, clamp.
  if (motor -> integrator_saturated){
    if (motor -> integral[1] > 0)
      motor -> integral[1] = integral_max;
    else
      motor -> integral[1] = -integral_max;
  }

  motor -> control[0] = motor -> control[1];

  motor -> control[1] = motor -> Kp * motor -> error[1] + motor -> Ki * motor -> integral[1] + motor -> Kd * motor -> filtered_derivative[1];

  // Check for control saturation.
  motor -> saturated = fabs(motor -> control[1]) >= 240;
  
  // If control is not saturated, continue as usual. Else, clamp.
  if (motor -> saturated){
    if (motor -> control[1] > 0)
      motor -> control[1] = 240;
    else
      motor -> control[1] = -240;
  }

  */

const float PWM_MAX = 255.0f;
const float dt = sampling_time / 1e6f;
const float kaw = 1.0f;  // anti-windup back-calc gain

// error
motor->error[0] = motor->error[1];
motor->error[1] = (target - motor->velocity[1]);

// PI-D (unsaturated)
float u_unsat = motor->Kp * motor->error[1]
              + motor->Ki * motor->integral[0]
              + motor->Kd * motor->filtered_derivative[1];

// saturate
float u = constrain(u_unsat, -PWM_MAX, PWM_MAX);
motor->saturated = (u != u_unsat);

// integrator with back-calculation
float e_avg = 0.5f * (motor->error[1] + motor->error[0]);
motor->integral[1] = motor->integral[0] + (e_avg + kaw * (u - u_unsat) / max(1e-6f, motor->Ki)) * dt;

// output
motor->control[0] = motor->control[1];
motor->control[1] = u;

motor->filtered_control[0] = motor->filtered_control[1];

motor->filtered_control[1] = motor -> b_c * motor->filtered_control[0] + motor -> a_c * (motor->control[1] + motor->control[0]);

motor->integral[0] = motor->integral[1];
  
}

void set_speed(Motor motor){
  analogWrite(motor.PWM, fabs(motor.filtered_control[1]));

  if (motor.LEFT)
    digitalWrite(motor.DIR, motor.filtered_control[1] < 0);
  else
    digitalWrite(motor.DIR, motor.filtered_control[1] > 0);
}

void compute_lowpass_filter_coeffs(Motor* motor, int sampling_time, float cutoff_freq){
  // y[n] = b * y[n-1] + a * (x[n] + x[n-1])

  motor -> b_c = ((1e6f/(float)sampling_time) - M_PI*cutoff_freq)/((1e6f/(float)sampling_time) + M_PI*cutoff_freq);
  motor -> a_c = cutoff_freq/((1e6f/((float)sampling_time*M_PI)) + cutoff_freq);

  motor -> b_d = ((1e6f/((float)sampling_time*velocity_window)) - M_PI*cutoff_freq)/((1e6f/((float)sampling_time*velocity_window)) + M_PI*cutoff_freq);
  motor -> a_d = cutoff_freq/((1e6f/((float)sampling_time*velocity_window*M_PI)) + cutoff_freq);
}

// Paired motor functions

void compute_speeds(){
  compute_speed(&left_motor);
  compute_speed(&right_motor);
}

void compute_controls(float left_target, float right_target){
  compute_control(&left_motor, left_target);
  compute_control(&right_motor, right_target);
}

void set_speeds(){
  set_speed(left_motor);
  set_speed(right_motor);
}

void print_RPMs(long int time){
  Serial.print(left_motor.RPM[1], 6);
  Serial.print(",");
  Serial.print(right_motor.RPM[1], 6);
  Serial.print(",");
  Serial.println(time);
}

void print_speeds(long int time){
  Serial.print(left_motor.velocity[1], 6);
  Serial.print(",");
  Serial.print(right_motor.velocity[1], 6);
  Serial.print(",");
  Serial.println(time);
}

void print_controls(long int time){
  Serial.print(left_motor.control[1], 6);
  Serial.print(",");
  Serial.print(right_motor.control[1], 6);
  Serial.print(",");
  Serial.println(time);
}

void print_speeds_and_controls(long int time){
  Serial.print(left_motor.velocity[1], 6);
  Serial.print(",");
  Serial.print(right_motor.velocity[1], 6);
  Serial.print(",");
  Serial.print(left_motor.filtered_control[1], 6);
  Serial.print(",");
  Serial.print(right_motor.filtered_control[1], 6);
  Serial.print(",");
  Serial.println(time);
}

void ResetPID(){
  left_motor.integral[0] = left_motor.integral[1] = 0;
  right_motor.integral[0] = right_motor.integral[1] = 0;

  left_motor.error[0] = left_motor.error[1] = 0;
  right_motor.error[0] = right_motor.error[1] = 0;

  left_motor.control[0] = left_motor.control[1] = 0;
  right_motor.control[0] = right_motor.control[1] = 0;

  left_motor.filtered_control[0] = left_motor.filtered_control[1] = 0;
  right_motor.filtered_control[0] = right_motor.filtered_control[1] = 0;

  left_motor.filtered_derivative[0] = left_motor.filtered_derivative[1] = 0;
  right_motor.filtered_derivative[0] = right_motor.filtered_derivative[1] = 0;
}

// ===== Wall PID Controller =====
float wall_pid(float base_speed, float left_dist, float dt){
  static float integral = 0.0f;
  static float prev_error = 0.0f;

  // --- PID gains ---
  const float Kp = 10.0f;
  const float Ki = 0.0f;
  const float Kd = 7.0f;

  // --- Logarithmic mapping for IR linearization ---
  float wall_norm = (float)left_dist;
  if (wall_norm < 30.0f) wall_norm = 30.0f;
  wall_norm = logf(wall_norm - 25.0f);

  float target_log = logf(150.0f - 25.0f);  // target further from wall
  float error = target_log - wall_norm;     // +ve = too far, -ve = too close

  // --- PID terms ---
  integral += error * dt;
  float derivative = (error - prev_error) / dt;
  prev_error = error;

  float correction = -(Kp * error + Kd * derivative + Ki * integral);
  correction = constrain(correction, -4.0f, 4.0f);

  return correction;
}



bool LeftWallPID(int dir, float dist){ // dir: 1 = forward, -1 = backward, dist in cm
  ResetPID();

  // zero encoder counts
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    left_motor.rotational_encoder_count  = 0;
    right_motor.rotational_encoder_count = 0;
  }

  done_follow = false;

  const float base_speed = 5.0f;       // cm/s
  const float target_norm = 0.5f;      // desired wall proximity (normalized)
  // const float max_output = 4.0f;       // max steering correction (cm/s)
  const float stop_front_threshold = 70.0f; // IR value to stop for front wall
  float left_targ, right_targ;

  unsigned long last_time_wall = micros();

  // encoder distance target
  float counts = 4.0f * (dist / wheel_circumference) * (left_motor.PPR);

  while (!done_follow)
  {
    current_time = micros();
    long elapsed_time = current_time - last_time;

    // --- wall detection ---
    detectWalls();  // updates front_dist, left_dist, right_dist

    // Stop if there is a wall directly in front
    if (front_dist >= stop_front_threshold) {
      set_targets(0, 0);
      compute_controls(0, 0);
      compute_speeds();
      set_speeds();
      done_follow = true;
      // turn(-90);
      Serial.println("Front wall confirmed — stopping");
      break;
    }

    // --- left-wall PID correction --- (only need this if there is a left wall to follow)
      unsigned long now = micros();
      float dt = (now - last_time_wall) / 1e6f;
      if (dt <= 0 || dt > 0.1f) dt = 0.01f;
      last_time_wall = now;

      float correction = wall_pid(base_speed, left_dist, dt);

      // --- Apply steering ---
      left_targ  = dir * (base_speed + correction);
      right_targ = dir * (base_speed - correction);

      // --- encoder stop condition ---
      int left_count, right_count;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        left_count = left_motor.rotational_encoder_count;
        right_count = right_motor.rotational_encoder_count;
      }

      if (fabs(left_count) >= counts || fabs(right_count) >= counts) {
        done_straight = true;
        left_targ = 0.0f;
        right_targ = 0.0f;
      }


    // --- PID and control update ---
    set_targets(left_targ, right_targ);

    if (elapsed_time >= sampling_time) {
      compute_controls(left_target, right_target);
      control_loops++;
      last_time = current_time;
    }

    if (control_loops >= velocity_window) {
      control_loops = 0;
      compute_speeds();
    }

    if (!done_follow)
      set_speeds();
  }

  // --- full stop ---
  set_targets(0, 0);
  compute_controls(0, 0);
  compute_speeds();
  set_speeds();
  delay(200);

  return done_follow;
}





bool straight(int dir, float dist){   // dir: 1 = forward, -1 = backward, dist in cm
  ResetPID();  // if you have a reset function for PID state
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
  left_motor.rotational_encoder_count  = 0;
  right_motor.rotational_encoder_count = 0;
  }

  done_straight = false;
  float left_targ  = (dir == 1) ? 4.0f : -4.0f;
  float right_targ = (dir == 1) ? 4.0f : -4.0f;

  float counts = 4.0f * (dist / wheel_circumference) * (left_motor.PPR);

  while (!done_straight)
  {
    current_time = micros();
    long elapsed_time = current_time - last_time;

    // --- Check wall ahead ---
    detectWalls();
    if (front_dist >= 100) {
      // wall detected — stop immediately
      set_targets(0, 0);
      compute_controls(0, 0);
      compute_speeds();
      set_speeds();
      delay(50);
      done_straight = true;
      return done_straight;
    }

    // --- Encoder-based distance stop (safety fallback) ---
    int left_count, right_count;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      left_count = left_motor.rotational_encoder_count;
      right_count = right_motor.rotational_encoder_count;
    }

    if (fabs(left_count) >= counts || fabs(right_count) >= counts) {
      done_straight = true;
      left_targ = 0.0f;
      right_targ = 0.0f;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        left_motor.rotational_encoder_count = 0;
        right_motor.rotational_encoder_count = 0;
      }
    }

    // --- PID update and control ---
    set_targets(left_targ, right_targ);

    if (elapsed_time >= sampling_time) {
      compute_controls(left_target, right_target);
      control_loops++;
      last_time = current_time;
    }

    if (control_loops >= velocity_window) {
      control_loops = 0;
      compute_speeds();
    }

    if (!done_straight) set_speeds();
  }

  // --- Stop fully ---
  set_targets(0, 0);
  compute_controls(0, 0);
  compute_speeds();
  set_speeds();
  delay(500);

  return done_straight;
}




bool turn(float degs){
  // --- setup ---
  ResetPID();
  done_turning = false;
  is_turning   = true;

  // Zero encoder counts at the start
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    left_motor.rotational_encoder_count  = 0;
    right_motor.rotational_encoder_count = 0;
  }

  const float K_TURN = 3.065f;   // geometric calibration constant (tune slightly)
  const float tol_deg = 0.20f;  // angular tolerance
  const float hold_s  = 0.06f;  // must stay within tol for 60 ms
  const float max_speed = 3.0f; // base turning speed (cm/s)
  const float min_speed = 0.6f; // minimum speed near stop
  const float ramp_zone = 20.0f; // start slowing within last 20°

  float settle_time = 0.0f;

  while (!done_turning)
  {
    current_time = micros();
    long elapsed_time = current_time - last_time;

    // --- read encoder angles ---
    int left_count, right_count;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      left_count  = left_motor.rotational_encoder_count;
      right_count = right_motor.rotational_encoder_count;
    }

    // Convert counts to degrees
    float left_degs  = (left_count  / (4.0f * left_motor.PPR))  * 360.0f / K_TURN;
    float right_degs = (right_count / (4.0f * right_motor.PPR)) * 360.0f / K_TURN;


    // Average heading and error
    float theta = 0.5f * (fabs(left_degs) + fabs(right_degs));
    Serial.print(theta, 3);
    Serial.println();
    float err   = fabs(degs) - theta;

    // --- proportional slow-down near target ---
    float scale = constrain(err / ramp_zone, 0.3f, 1.0f);
    float base  = max_speed * scale;
    base = (base < min_speed) ? min_speed : base;

    float left_targ, right_targ;
    if (degs > 0) {  // turn left (CCW)
      left_targ  = -base;
      right_targ =  base;
    } else {          // turn right (CW)
      left_targ  =  base;
      right_targ = -base;
    }

    // --- check tolerance + settle window ---
    if (fabs(err) <= tol_deg) {
      settle_time += (elapsed_time * 1e-6f);
    } else {
      settle_time = 0.0f;
    }

    if (settle_time >= hold_s) {
      done_turning = true;
      is_turning   = false;
      left_targ = right_targ = 0.0f;

      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        left_motor.rotational_encoder_count  = 0;
        right_motor.rotational_encoder_count = 0;
      }
    }

    // --- PID control ---
    set_targets(left_targ, right_targ);

    if (elapsed_time >= sampling_time) {
      compute_controls(left_target, right_target);
      control_loops++;
      last_time = current_time;
    }

    if (control_loops >= velocity_window) {
      control_loops = 0;
      compute_speeds();
    }

    if (!done_turning) set_speeds();
  }

  // --- full stop ---
  set_targets(0, 0);
  compute_controls(0, 0);
  compute_speeds();
  set_speeds();
  delay(1000);

  return done_turning;
}



void set_targets(float left, float right){
  left_target = left;
  right_target = right;
}

// ISR

void readLeftEncoderA(){
  static int last_count_time = 0;
  if (micros() - last_count_time < debounce_time)
    return;
  static bool old_AxorB = false;
  static bool old_B = false;
  bool new_AxorB = digitalReadFast(left_motor.ENCODER_AxorB);
  bool new_B = digitalReadFast(left_motor.ENCODER_B);
  if ((old_AxorB ^ new_B) != (new_AxorB ^ old_B)){
    left_motor.encoder_count--;
    left_motor.rotational_encoder_count--;
  }
  else{
    left_motor.encoder_count++;
    left_motor.rotational_encoder_count++;
  }

  old_AxorB = new_AxorB;
  old_B = new_B;
  
  last_count_time = micros();
}


void readRightEncoderA(){
  static int last_count_time = 0;
  if (micros() - last_count_time < debounce_time)
    return;
  static bool old_AxorB = false;
  static bool old_B = false;
  bool new_AxorB = digitalReadFast(right_motor.ENCODER_AxorB);
  bool new_B = digitalReadFast(right_motor.ENCODER_B);
  if ((old_AxorB ^ new_B) != (new_AxorB ^ old_B)){
    right_motor.encoder_count++;
    right_motor.rotational_encoder_count++;
  }
  else{
    right_motor.encoder_count--;
    right_motor.rotational_encoder_count--;
  }

  old_AxorB = new_AxorB;
  old_B = new_B;

  last_count_time = micros();
}

// Sensor Functions
int sensorDark  = 0;
int sensorLit   = 0;
int sensorValue = 0;

// === Sensors ===
void init_sensors() {


  pinMode(EMITTER, OUTPUT);
  digitalWrite(EMITTER, LOW);  // ensure off

  pinMode(SENSOR_RIGHT_MARK, INPUT);
  pinMode(SENSOR_1, INPUT);
  pinMode(SENSOR_2, INPUT);
  pinMode(SENSOR_3, INPUT);
  pinMode(SENSOR_4, INPUT);
  pinMode(SENSOR_LEFT_MARK, INPUT);

  delay(50);
}

// === Single-sensor read with emitter gating ===
int readSensor(int channel)
{
  // Measure ambient light first (emitter off)
  digitalWrite(EMITTER, LOW);
  delayMicroseconds(100);
  int dark = analogRead(channel);

  // Turn emitter on and measure reflected light
  digitalWrite(EMITTER, HIGH);
  delayMicroseconds(150);  // allow IR LED to stabilize
  int lit = analogRead(channel);

  // Turn emitter off
  digitalWrite(EMITTER, LOW);

  // Compute reflection difference (lit - dark)
  int diff = lit - dark;
  if (diff < 0) diff = 0;
  if (diff > 1023) diff = 1023;

  return diff;
}

// === Sequential multi-sensor read ===
void readAll()
{
  // Each sensor read happens in isolation to avoid optical crosstalk
  left_dist  = readSensor(A2);  // left side
  // delayMicroseconds(200);
  Serial.print("left dist: ");
  Serial.print(left_dist);

  right_dist = readSensor(A1);  // right side
  // delayMicroseconds(200);
  Serial.print(" right dist:");
  Serial.print(right_dist);

  front_dist = readSensor(A0);  // front side
  // delayMicroseconds(200);
  Serial.print(" front dist: ");
  Serial.print(front_dist);
  Serial.println();
}


// int readFront(){
//   int left_reflectf  = readSensor(A0);
//   int right_reflectf = readSensor(A3);
//   int diffF = abs(left_reflectf - right_reflectf);
//   return diffF;
// }

// int readRight(){
//   int left_reflectr  = readSensor(A1);
//   int right_reflectr = readSensor(A4);
//   int diffR = abs(left_reflectr - right_reflectr);
//   return diffR;
// }

// int readLeft(){
//   int left_reflectr  = readSensor(A2);
//   int right_reflectr = readSensor(A5);
//   int diffR = abs(left_reflectr - right_reflectr);
//   return diffR;
// }

int front_dist = 0;
int right_dist = 0;
int left_dist = 0;

  bool wall_front = false;
  bool wall_right = false;
  bool wall_left = false;

void detectWalls(){


  int L_threshold = 100;
  int R_threshold = 100;
  int F_threshold = 70;

  wall_front = false;
  wall_right = false;
  wall_left = false;

  readAll();

  if (front_dist >= F_threshold) 
  {
    wall_front = true;
    Serial.print("Front Wall Found ");
    Serial.print(front_dist);
    Serial.println();
  }
  if (left_dist >= L_threshold)
  {
    wall_left = true;
    Serial.print("Left Wall Found ");
    Serial.print(left_dist);
    Serial.println();
  }
  if (right_dist >= R_threshold)
  {
    wall_right = true;
    Serial.print("Right Wall Found ");
    Serial.print(right_dist);
    Serial.println();
  }
}

// void LeftWallFollower() {
//   detectWalls();
//   Serial.print("WF: ");
//   Serial.print(wall_front);
//   Serial.print(" WR: ");
//   Serial.print(wall_right);
//   Serial.print(" WL: ");
//   Serial.print(wall_left);
//   Serial.println();

//   if ((wall_left && wall_front && !wall_right) || (wall_front && !wall_left && !wall_right)) 
//   {
//     turn(-90);            // no wall on right → turn right
//   }
//   else if (wall_front && wall_right && !wall_left ) 
//   {
//     turn(90);             // wall in front → turn left
//   }
//   else if((wall_left && !wall_right) || (wall_left && wall_right))
//   {
//     LeftWallPID(1, 999.0);
//   }
//   else
//   {
//     straight(1, 999.0);
//   }
    
// }






