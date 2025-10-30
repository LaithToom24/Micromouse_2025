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

bool turn(float degs){
  /*
  static long int now = micros();
  static float reached = false;
  static float robo_degs = 0;
  if (!reached)
    robo_degs = 360.0f * 5.0f * (micros() - now) / (M_PI * 8.25e6f); 

  Serial.println(robo_degs, 4);

  if (fabs(robo_degs - degs) < 0.5f){
    set_targets(0, 0);
    reached = true;
  }
  else if (degs > 0)
    set_targets(-5, 5);
  else
    set_targets(5, -5);
  */
  static float last_degs = degs;
  float left_targ, right_targ;
  static bool rotation_done = false;
  bool new_turn = (last_degs != degs);
  Serial.println(new_turn);

  if (new_turn){
    rotation_done = false;
    done_turning = false;
    is_turning = true;
  }
  
  if (!rotation_done){
    if (degs > 0){
      //is_turning = true;
      left_targ = -1.5;
      right_targ = 1.5;
    }
    else{
      //is_turning = true;
      left_targ = 1.5;
      right_targ = -1.5;
    }
  }

  int left_count, right_count;
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE){
    if (abs(left_motor.rotational_encoder_count) > 4 * left_motor.PPR)
      left_motor.rotational_encoder_count = 0;
    left_count = left_motor.rotational_encoder_count;

    if (abs(right_motor.rotational_encoder_count) > 4 * right_motor.PPR)
      right_motor.rotational_encoder_count = 0;
    right_count = right_motor.rotational_encoder_count;
  }
  float left_degs = (left_count/(4 * left_motor.PPR)) * 360.0f;
  float right_degs = (right_count/(4 * right_motor.PPR)) * 360.0f;

    if (rotation_done || fabs(fabs(left_degs) - fabs(degs)) < 1.5 || fabs(fabs(right_degs) - fabs(degs)) < 1.5){
      rotation_done = true;
    done_turning = true;
    is_turning = false;
    left_targ = 0;
    right_targ = 0;
    ATOMIC_BLOCK (ATOMIC_RESTORESTATE){
      left_motor.rotational_encoder_count = 0;
      right_motor.rotational_encoder_count = 0;
    }
  }

  set_targets(left_targ, right_targ);

  last_degs = degs;

  return rotation_done;
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


