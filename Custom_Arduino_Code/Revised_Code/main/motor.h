//#include <digitalWriteFast.h>
#include <util/atomic.h>
#include "PID_controller.h"
#include <stdbool.h>
#include <stdint.h>

class Motor
{
  public: 

  Motor(int enb, int dir, int encoder_axorb, int encoder_b, int cpr, float gear_ratio, float wheel_diameter, bool left, float kf_v, float kp_v, float ki_v, float kd_v, float kp_p, float kf_p, float ki_p, float kd_p, int control_period, int velocity_period, int position_period, float cutoff_freq, float vel_maximum);
  void init(void (*isr));
  void set_vel(float vel);
  void set_pos(float pos);
  float get_vel();
  float get_pos();
  float get_pwm();
  void readEncoder();

  private:

  float update_vel();
  float update_pos();

  // useful values
  float feedforward_gain_v = 0.0f;
  float feedforward_gain_p = 0.0f;
  float vel_max = 16.0f;
  float count_to_vel;

  // pins
  int ENB;
  int DIR;
  int dir_bit;
  int ENCODER_AxorB;
  int encoderAxorB_bit;
  int ENCODER_B;
  int encoderB_bit;

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
  volatile int rotational_encoder_count;
  volatile unsigned long last_count_time;
  volatile bool old_AxorB;
  volatile bool old_B;
  float dcount[2] = {0};
  float dcount_avg[2] = {0};
  // signed pwm value to write to motor (sign represents direction)
  float pwm = 0;
  float last_pwm = 0;
  int8_t last_dir = 0;
  unsigned long last_control_time = 0;

  // motor orientation
  bool LEFT; 

  // motor controllers
  PID_Controller velocity_controller;
  PID_Controller position_controller;
  
  // controller settings
  int ctrl_period;
  int vel_period;
  int pos_period;

};

Motor::Motor(int enb, int dir, int encoder_axorb, int encoder_b, int cpr, float gear_ratio, float wheel_diameter, bool left, float kf_v, float kp_v, float ki_v, float kd_v, float kp_p, float kf_p, float ki_p, float kd_p, int control_period, int velocity_period, int position_period, float cutoff_freq, float vel_maximum){
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

  CPR = cpr * gear_ratio;
  PPR = 4 * CPR;
  LEFT = left;

  wheel_circumference = wheel_diameter * M_PI; 

  ctrl_period = control_period;
  vel_period = velocity_period;
  pos_period = position_period;

  velocity_controller.setup(kp_v, ki_v, kd_v, control_period, velocity_period, cutoff_freq, 255.0f);
  position_controller.setup(kp_p, ki_p, kd_p, control_period, position_period, cutoff_freq, vel_max);

  velocity = 0.0f;
  position = 0.0f;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    encoder_count = 0;
    rotational_encoder_count = 0;
    last_count_time = 0;
    old_AxorB = false;
    old_B = false;
  }

  feedforward_gain_v = kf_v * 255.0f/vel_max; 
  vel_max = vel_maximum;
  count_to_vel = 3.0f * wheel_circumference * 1e6f / (PPR * (float)vel_period * (float)ctrl_period);

  feedforward_gain_p = kf_p * vel_max / 360.0f;
}

void Motor::init(void (*isr)){
  pinMode(ENB, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(ENCODER_AxorB, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_AxorB), isr, CHANGE);
}

void Motor::set_vel(float vel){
  unsigned long now = micros();

  vel = constrain(vel, -vel_max, vel_max);

  // feedforward control
  pwm = vel * feedforward_gain_v;

  // feedback control with deadsetting
  if (fabs(vel) > 0.1){
    pwm += velocity_controller.process(vel - velocity, velocity, now);
    //Serial.println(pwm);
  }
  else
    velocity_controller.process(now);

  pwm = constrain(pwm, -255.0f, 255.0f);

  bitWrite(*dir_port, dir_bit, (LEFT ^ (pwm > 0)));
  analogWrite(ENB, fabs(pwm));

  if (velocity_controller.get_control_loops() == vel_period)
    velocity = update_vel();
}

void Motor::set_pos(float pos){
  unsigned long now = micros();

  pos = constrain(pos, -360.0f, 360.0f);

  float vel = pos * feedforward_gain_p;

  float error = pos - position;

  if (fabs(error) > 10.0f)
    vel += position_controller.process(pos - position, position, now);
  else{
    position_controller.process(now);
    vel = 0;
  }

  vel = constrain(vel, -vel_max, vel_max);

  set_vel(vel);

  if (position_controller.get_control_loops() == pos_period)
    position = update_pos();
}

float Motor::update_vel(){
  float vel;

  dcount[0] = dcount[1];
  //unsigned long dcount_num;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    //dcount[1] = encoder_count;
    dcount[1] = encoder_count;
    // update last encoder count
    encoder_count = 0;
  }
  
  dcount_avg[0] = dcount_avg[1];
  if (pwm != 0.0f)
    dcount_avg[1] = 0.9875f * dcount_avg[0] + 0.0062f * (dcount[1] + dcount[0]);
  else
    dcount_avg[1] = (dcount[1] + dcount[0]) * 0.5f;
    //dcount_avg[1] = -1.0f * dcount_avg[0] + 1.0f * (dcount[1] + dcount[0]);

  vel = last_dir * count_to_vel * dcount_avg[1];

  //Serial.println(dcount_avg[2], 3);

  if (pwm != 0.0f)
    last_dir = (LEFT ^ bitRead(*dir_pin, dir_bit)) ? 1 : -1;
  else
    last_dir = 0;

  return vel;
}

float Motor::update_pos(){
  float pos;

  long rcount;
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE){
    rcount = rotational_encoder_count;
    if (rotational_encoder_count >= 8*PPR)
      rotational_encoder_count = 0;
  }

  pos = last_dir * (rcount * 360.0f / (8*PPR));

  return pos;
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

void Motor::readEncoder(){
  unsigned long now = micros();

  // debounce 
  if (now - last_count_time < 7)
    return;

  bool new_AxorB = bitRead(*encoderAxorB_pin, encoderAxorB_bit);
  bool new_B = bitRead(*encoderB_pin, encoderB_bit);

  if (new_AxorB != old_AxorB || old_B != new_B){
    encoder_count++;
    rotational_encoder_count++;
  }

  //encoder_count++;
  //rotational_encoder_count++;

  /*
  if ((old_AxorB ^ new_B) != (new_AxorB ^ old_B)){
    if (LEFT){
      encoder_count--;
      rotational_encoder_count--;
    }
    else{
      encoder_count++;
      rotational_encoder_count++;
    }
  }
  else{
    if (LEFT){
      encoder_count++;
      rotational_encoder_count++;
    }
    else{
      encoder_count--;
      rotational_encoder_count--;
    }
  }
  */

  old_AxorB = new_AxorB;
  old_B = new_B;

  last_count_time = now;
}
