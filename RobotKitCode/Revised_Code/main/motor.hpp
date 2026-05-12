#include "PID_controller.hpp"

class Motor
{
  public: 

  Motor();
  // motor initialization
  void set_motor_pins(int enb, int dir, int encoder_axorb, int encoder_b);
  void set_interrupt(void (*isr_AxorB)());
  void set_motor_specs(int cpr, float gear_ratio, float wheel_diameter, float vel_maximum);
  void set_velocity_controls(float kf, float kp, float ki, float kd, int control_period, int measurement_rate, float cutoff_freq);
  void set_motor_orientation(bool left);

  // setter and getter functions
  // the setter function for velocity implements PID control
  void set_vel(float vel);
  void reset_position();
  void reset_velocity();
  void reset_velocity_controller();
  float get_vel();
  float get_pos();
  float get_pwm();
  float get_distance();

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