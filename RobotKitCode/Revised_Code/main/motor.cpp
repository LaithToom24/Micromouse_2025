#include <Arduino.h>
#include "motor.hpp"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <digitalWriteFast.h>

// constructor
Motor::Motor(){
  velocity = 0.0f;
  position = 0.0f;

  //ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    noInterrupts();
    encoder_count = 0;
    rotational_encoder_count = 0;
    last_count_time = 0;
    old_B = false;
    old_AxorB = false;
    interrupts();
  //}
}

// initialization functions
void Motor::set_motor_pins(int enb, int dir, int encoder_axorb, int encoder_b){
  ENB = enb;
  DIR = dir;
  ENCODER_AxorB = encoder_axorb;
  ENCODER_B = encoder_b;

  pinMode(ENB, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(ENCODER_AxorB, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
}

void Motor::set_interrupt(void (*isr_AxorB)()){
  attachInterrupt(digitalPinToInterrupt(ENCODER_AxorB), isr_AxorB, CHANGE);
}

void Motor::set_motor_specs(int cpr, float gear_ratio, float wheel_diameter, float vel_maximum){
  CPR = cpr * gear_ratio;
  PPR = 4 * CPR;
  wheel_circumference = wheel_diameter * M_PI; 
  vel_max = vel_maximum;
}

void Motor::set_velocity_controls(float kf, float kp, float ki, float kd, int control_period, int measurement_rate, float cutoff_freq){
  feedforward_gain_v = kf * 255.0f/vel_max; 
  velocity_controller.setup(kp, ki, kd, control_period, measurement_rate, cutoff_freq, 255.0f);
  vel_rate = measurement_rate;
  vel_ctrl_period = control_period;
  count_to_vel = 2.0f * wheel_circumference * 1e6f / (PPR * (float)vel_rate * (float)vel_ctrl_period);
}

void Motor::set_motor_orientation(bool left){
  LEFT = left;
}

// main loop functions
void Motor::set_vel(float vel){
  unsigned long now = micros();

  vel = constrain(vel, -vel_max, vel_max);

  // feedforward control
  pwm = vel * feedforward_gain_v;

  // feedback control with deadsetting
  if (fabs(vel) > 0.1){
    pwm += velocity_controller.process(vel - velocity, velocity, now);
    pwm = constrain(pwm, -255.0f, 255.0f);
  }
  else{
    velocity_controller.process(now);
    velocity_controller.clear();
    pwm = 0;
  }

  digitalWriteFast(DIR, (LEFT ^ (pwm > 0)));
  analogWrite(ENB, fabs(pwm));

  if (velocity_controller.get_control_loops() == vel_rate)
    velocity = update_vel();
}

float Motor::update_vel(){
  float vel;

  dcount[0] = dcount[1];;
  //ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    noInterrupts();
    dcount[1] = encoder_count;
    // update last encoder count
    encoder_count = 0;
    interrupts();
  //}
  
  /*
  dcount_avg[0] = dcount_avg[1];
  if (pwm != 0.0f)
    dcount_avg[1] = 0.85f * dcount_avg[0] + 0.07f * (dcount[1] + dcount[0]);
  else
    dcount_avg[1] = (dcount[1] + dcount[0]) * 0.5f;
  */

  vel = count_to_vel * dcount[1];

  if (fabs(vel) > vel_max)
    return velocity;

  return vel;
}

void Motor::reset_velocity(){
  noInterrupts();
  encoder_count = 0;
  interrupts();
  //}
  velocity = 0.0f;
}

void Motor::reset_velocity_controller(){
  velocity_controller.clear();
}

void Motor::update_pos(){
  long rcount;
  //ATOMIC_BLOCK (ATOMIC_RESTORESTATE){
    noInterrupts();
    rcount = rotational_encoder_count;
    interrupts();
  //}

  position = (rcount * 360.0f / (10.0f*float(PPR)));
}

float Motor::get_vel(){
  return velocity;
}

float Motor::get_pos(){
  return position;
}

float Motor::get_pwm(){
  return pwm;
}

bool Motor::is_turning(){
  return isTurning;
}

void Motor::reset_position(){
  //ATOMIC_BLOCK (ATOMIC_RESTORESTATE){
    noInterrupts();
    rotational_encoder_count = 0;
    interrupts();
  //}
  position = 0.0f;
}

void Motor::readEncoder(){
  bool new_B = digitalReadFast(ENCODER_B);
  bool new_AxorB = digitalReadFast(ENCODER_AxorB);

  uint8_t newState = (new_B << 1) | new_AxorB;

  uint8_t oldState = (old_B << 1) | old_AxorB;

  if ((oldState == 0b00 && newState == 0b01) || (oldState == 0b01 && newState == 0b10) || (oldState == 0b10 && newState == 0b11) || (oldState == 0b11 && newState == 0b00)){
    if (LEFT){
      encoder_count++;
      rotational_encoder_count++;
    }
    else{
      encoder_count--;
      rotational_encoder_count--;
    }
  }
  else if ((oldState == 0b00 && newState == 0b11) || (oldState == 0b11 && newState == 0b10) || (oldState == 0b10 && newState == 0b01) || (oldState == 0b01 && newState == 0b00)){
    if (LEFT){
      encoder_count--;
      rotational_encoder_count--;
    }
    else{
      encoder_count++;
      rotational_encoder_count++;
    }
  }

  old_B = new_B;
  old_AxorB = new_AxorB;
}

float Motor::get_distance(){
  long rcount;
  
  // Safely grab the count without interrupts messing it up
  noInterrupts();
  rcount = rotational_encoder_count;
  interrupts();
  
  // Calculate distance: (Total Ticks / Ticks Per Revolution) * Circumference
  return ((float)rcount / PPR) * wheel_circumference;
}
