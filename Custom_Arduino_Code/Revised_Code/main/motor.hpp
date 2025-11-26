#include <util/atomic.h>
#include "PID_controller.hpp"
#include <stdbool.h>
#include <stdint.h>

class Motor
{
  public: 

  Motor();
  // motor initialization
  void set_motor_pins(int enb, int dir, int encoder_axorb, int encoder_b);
  void set_interrupt(void (*isr_AxorB));
  void set_motor_specs(int cpr, float gear_ratio, float wheel_diameter, float vel_maximum);
  void set_velocity_controls(float kf, float kp, float ki, float kd, int control_period, int measurement_rate, float cutoff_freq);
  void set_motor_orientation(bool left);

  // setter and getter functions
  // the setter function for velocity implements PID control
  void set_vel(float vel);
  void reset_position();
  float get_vel();
  float get_pos();
  float get_pwm();

  // measurement functions
  float update_vel();
  void update_pos();

  // returns whether or not the motor is part of a turn command
  bool is_turning();

  // read the encoder; meant to be the ISR for the encoder pin
  void readEncoder();

  private:

  // useful values
  float feedforward_gain_v = 0.0f;
  float vel_max = 16.0f;
  float count_to_vel;
  float prev_vel = 0.0f;

  // pins
  short ENB;
  short DIR;
  short dir_bit;
  short ENCODER_AxorB;
  short encoderAxorB_bit;
  short ENCODER_B;
  short encoderB_bit;

  // ports and pins
  volatile uint8_t *dir_port;
  volatile uint8_t *dir_pin;
  volatile uint8_t *pwm_port;
  volatile uint8_t *encoderAxorB_pin;
  volatile uint8_t *encoderB_pin;

  // motor specs
  float CPR;
  // effective CPR for motor
  float PPR;
  // effective PPR for motor
  float wheel_circumference;

  // motor stats
  float position = 0;
  float velocity = 0;
  volatile long encoder_count;
  volatile long rotational_encoder_count;
  volatile unsigned long last_count_time;
  float dcount[2] = {0};
  float dcount_avg[2] = {0};
  // signed pwm value to write to motor (sign represents direction)
  float pwm = 0;
  unsigned long last_control_time = 0;
  volatile bool old_B;
  volatile bool old_AxorB;
  bool isTurning = false;

  // motor orientation
  bool LEFT; 

  // motor controllers
  PID_Controller velocity_controller;
  
  // controller settings
  int vel_ctrl_period;
  int vel_rate;

};

// constructor
Motor::Motor(){
  velocity = 0.0f;
  position = 0.0f;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    encoder_count = 0;
    rotational_encoder_count = 0;
    last_count_time = 0;
    old_B = false;
    old_AxorB = false;
  }
}

// initialization functions
void Motor::set_motor_pins(int enb, int dir, int encoder_axorb, int encoder_b){
  ENB = enb;
  DIR = dir;
  ENCODER_AxorB = encoder_axorb;
  ENCODER_B = encoder_b;

  if (0 <= dir && dir <= 7){
    dir_port = &PORTD;
    dir_pin = &PIND;
    dir_bit = DIR;
  }
  else if (8 <= dir && dir <= 13){
    dir_port = &PORTB;
    dir_pin = &PINB;
    dir_bit = DIR - 8;
  }

  if (0 <= ENCODER_AxorB && ENCODER_AxorB <= 7){
    encoderAxorB_pin = &PIND;
    encoderAxorB_bit = ENCODER_AxorB;
  }
  else if (8 <= ENCODER_AxorB && ENCODER_AxorB <= 13){
    encoderAxorB_pin = &PINB;
    encoderAxorB_bit = ENCODER_AxorB - 8;
  }

  if (0 <= ENCODER_B && ENCODER_B <= 7){
    encoderB_pin = &PIND;
    encoderB_bit = ENCODER_B;
  }
  else if (8 <= ENCODER_B && ENCODER_B <= 13){
    encoderB_pin = &PINB;
    encoderB_bit = ENCODER_B - 8; 
  }

  pinMode(ENB, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(ENCODER_AxorB, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
}

void Motor::set_interrupt(void (*isr_AxorB)){
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
  count_to_vel = 1.5f * wheel_circumference * 1e6f / (CPR * (float)vel_rate * (float)vel_ctrl_period);
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
  if (fabs(vel) > 0.1)
    pwm += velocity_controller.process(vel - velocity, velocity, now);
  else{
    velocity_controller.process(now);
    velocity_controller.clear();
  }

  pwm = constrain(pwm, -255.0f, 255.0f);

  bitWrite(*dir_port, dir_bit, (LEFT ^ (pwm > 0)));
  analogWrite(ENB, fabs(pwm));

  if (velocity_controller.get_control_loops() == vel_rate)
    velocity = update_vel();
}

float Motor::update_vel(){
  float vel;

  dcount[0] = dcount[1];;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    dcount[1] = encoder_count;
    // update last encoder count
    encoder_count = 0;
  }
  
  dcount_avg[0] = dcount_avg[1];
  if (pwm != 0.0f)
    dcount_avg[1] = 0.8818f * dcount_avg[0] + 0.0591f * (dcount[1] + dcount[0]);
  else
    dcount_avg[1] = (dcount[1] + dcount[0]) * 0.5f;

  vel = count_to_vel * dcount_avg[1];

  if (fabs(vel) > vel_max)
    return velocity;

  return vel;
}

void Motor::update_pos(){
  long rcount;
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE){
    rcount = rotational_encoder_count;
  }

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
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE){
    rotational_encoder_count = 0;
  }
  position = 0.0f;
}

void Motor::readEncoder(){
  // debounce 
  delayMicroseconds(10);

  bool new_B = bitRead(*encoderB_pin, encoderB_bit);
  bool new_AxorB = bitRead(*encoderAxorB_pin, encoderAxorB_bit);

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
