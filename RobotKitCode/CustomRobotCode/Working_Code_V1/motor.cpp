#include <Arduino.h>
#include "motor.hpp"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

// constructor
Motor::Motor(){
  velocity = 0.0f;
  position = 0.0f;

  //ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    noInterrupts();
    encoder_count = 0;
    rotational_encoder_count = 0;
    last_count_time = 0;
    interrupts();
  //}
}

// initialization functions
void Motor::set_motor_pins(int pwm_pin, int in1, int in2, int encoder_a, int encoder_b){
  ENB = pwm_pin;
  IN1 = in1;
  IN2 = in2;
  ENCODER_A = encoder_a;
  ENCODER_B = encoder_b;

  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
}

void Motor::set_interrupt(void (*isr_A)()){
  // Trigger the interrupt whenever Channel A changes state (RISING or FALLING)
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), isr_A, CHANGE);
}

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

  // Directional Logic for IN1/IN2 Driver
  bool forward = (pwm > 0);
  
  // Flip directional logic if it's the right-side motor
  if (!LEFT) {
      forward = !forward;
  }

  //if (fabs(pwm) < 0.1) {
  //    // Coast / Stop
  //    digitalWrite(IN1, LOW);
  //    digitalWrite(IN2, LOW);

  if (fabs(pwm) < 0.7) {
      // Coast / Stop
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, HIGH);
  } else if (forward) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
  } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
  }

  // Write the PWM speed
  analogWrite(ENB, fabs(pwm));

  if (velocity_controller.get_control_loops() == vel_rate)
    velocity = update_vel();
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
  // Read the current state of both pins
  bool new_A = digitalRead(ENCODER_A);
  bool new_B = digitalRead(ENCODER_B);

  // 2x Quadrature Decoding Logic
  // If A and B match after A changes, we are spinning one way. 
  // If they are opposite, we are spinning the other way.
  if (new_A == new_B) {
    if (LEFT) {
      encoder_count++;
      rotational_encoder_count++;
    } else {
      encoder_count--;
      rotational_encoder_count--;
    }
  } else {
    if (LEFT) {
      encoder_count--;
      rotational_encoder_count--;
    } else {
      encoder_count++;
      rotational_encoder_count++;
    }
  }
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
