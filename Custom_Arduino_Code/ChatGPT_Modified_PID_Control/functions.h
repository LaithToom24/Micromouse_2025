#include "declarations.h"

// Individual motor functions

void init_motor(Motor* motor, unsigned short PWM, unsigned short DIR, unsigned short ENCODER_A, unsigned short ENCODER_B, int specified_CPR, float gear_ratio, bool LEFT){
  motor -> PWM = PWM;
  motor -> DIR = DIR;
  motor -> ENCODER_A = ENCODER_A;
  motor -> ENCODER_B = ENCODER_B;
  motor -> specified_CPR = specified_CPR;
  motor -> gear_ratio = gear_ratio;
  motor -> CPR = specified_CPR * gear_ratio;
  motor -> PPR = 4 * motor -> CPR;
  motor -> LEFT = LEFT;

  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(ENCODER_A, INPUT_PULLUP);
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
  float CPR = motor -> CPR;
  int dcount = (motor -> encoder_count[1] - motor -> encoder_count[0]);
  motor -> encoder_count[0] = motor -> encoder_count[1];

  float revolutions = dcount / CPR;
  float RPM = (60*1000000) * revolutions / velocity_window;
  float velocity = wheel_circumference * RPM / 60; 

  motor -> RPM[1] = RPM;
  motor -> velocity[1] = velocity;

  // low-pass filter for velocity measurement
  motor -> filtered_velocity[1] = 0.9691 * motor -> filtered_velocity[0] + 0.0155 * (motor -> velocity[1] + motor -> velocity[0]);

  motor -> derivative[1] = 2 * (motor -> Kd) * (1.0 / (float)sampling_time) * (motor -> filtered_velocity[1] - motor -> filtered_velocity[0]) - motor -> derivative[0]; 

  // low-pass filter for derivative term
  motor -> filtered_derivative[1] = 0.2283 * motor -> filtered_derivative[0] + 0.3859 * (motor -> derivative[1] + motor -> derivative[0]);

  // update last encoder count
}

void compute_control(Motor* motor, float target){
  if (fabs(target) > 7)
    target = -1 * (target < 0) * 7;
  else if (fabs(target) < 0.1){
    motor -> control[1] = 0;
    return;
  }
  motor -> error[0] = motor -> error[1];
  motor -> error[1] = target - motor -> filtered_velocity[1];

  if (!motor -> saturated){
    motor -> integral[1] = motor -> integral[0] + 0.5 * motor -> Ki * sampling_time * (motor -> error[1] + motor -> error[0]);
  }

  motor -> control[0] = motor -> control[1];
  motor -> control[1] = motor -> Kp * motor -> error[1] + motor -> Ki * motor -> integral[1] * !motor -> saturated + motor -> Kd * motor -> filtered_derivative[1];

  motor -> saturated = motor -> control[1] >= 240;
}

void set_speed(Motor motor){
  if (!motor.saturated)
    digitalWrite(motor.PWM, fabs(motor.control[1]));
  else
    digitalWrite(motor.PWM, 240);
  if (motor.LEFT)
    digitalWrite(motor.DIR, motor.control[1] < 0);
  else
    digitalWrite(motor.DIR, motor.control[1] > 0);
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

void print_speeds(long int time){
  Serial.print(left_motor.filtered_velocity[1], 6);
  Serial.print(",\t");
  Serial.print(right_motor.filtered_velocity[1], 6);
  Serial.print(",\t");
  Serial.println(time);
}

void print_RPMs(long int time){
  Serial.print(left_motor.RPM[1], 6);
  Serial.print(",\t");
  Serial.print(right_motor.RPM[1], 6);
  Serial.print(",\t");
  Serial.println(time);
}

void print_controls(long int time){
  Serial.print(left_motor.control[1], 6);
  Serial.print(",\t");
  Serial.print(right_motor.control[1], 6);
  Serial.print(",\t");
  Serial.println(time);
}

// ISR

void readLeftEncoderA(){
  static unsigned long last_time = 0;
  unsigned long now = micros();
  if (now - last_time < 200) return; // debounce 200us
  last_time = now;

  /*
  left_motor.encoder_count[1] += (digitalRead(left_motor.ENCODER_A) && digitalRead(left_motor.ENCODER_B));
  left_motor.encoder_count[1] -= (digitalRead(left_motor.ENCODER_A) || digitalRead(left_motor.ENCODER_B));
  */
  /*
  left_motor.phaseA[0] = left_motor.phaseA[1];
  left_motor.phaseB[0] = left_motor.phaseB[1];
  left_motor.phaseA[1] = digitalRead(left_motor.ENCODER_A);
  left_motor.phaseB[1] = digitalRead(left_motor.ENCODER_B);

  int encoded = (left_motor.phaseA[1] << 1) | left_motor.phaseB[1];
  int sequence = (left_motor.phaseA[0] << 2) | encoded;
  //int current_pair = (phaseA << 1) + phaseB;
  //int past_pair = (right_phaseA << 1) + right_phaseB;
  //int sequence = (past_pair << 2) + current_pair;

  if (sequence == 0b0001 || sequence == 0b0111 || sequence == 0b1110 || sequence == 0b1000)
    left_motor.encoder_count[1]++;
  else if (sequence == 0b0010 || sequence == 0b1011 || sequence == 0b1101 || sequence == 0b0100)
    left_motor.encoder_count[1]--;
  return;
  */
  if (digitalRead(left_motor.ENCODER_B) == HIGH){
    if (digitalRead(left_motor.ENCODER_A) == HIGH)
      left_motor.encoder_count[1]++;
    else
      left_motor.encoder_count[1]--;
  }
  else{
    if (digitalRead(left_motor.ENCODER_A) == HIGH)
      left_motor.encoder_count[1]--;
    else
      left_motor.encoder_count[1]++;
  }
  return;
}

void readLeftEncoderB(){
  /*
  left_motor.encoder_count[1] += (digitalRead(left_motor.ENCODER_A) && digitalRead(left_motor.ENCODER_B));
  left_motor.encoder_count[1] -= (digitalRead(left_motor.ENCODER_A) || digitalRead(left_motor.ENCODER_B));
  */
  /*
  left_motor.phaseA[0] = left_motor.phaseA[1];
  left_motor.phaseB[0] = left_motor.phaseB[1];
  left_motor.phaseA[1] = digitalRead(left_motor.ENCODER_A);
  left_motor.phaseB[1] = digitalRead(left_motor.ENCODER_B);

  int encoded = (left_motor.phaseA[1] << 1) | left_motor.phaseB[1];
  int sequence = (left_motor.phaseA[0] << 2) | encoded;
  //int current_pair = (phaseA << 1) + phaseB;
  //int past_pair = (right_phaseA << 1) + right_phaseB;
  //int sequence = (past_pair << 2) + current_pair;

  if (sequence == 0b0001 || sequence == 0b0111 || sequence == 0b1110 || sequence == 0b1000)
    left_motor.encoder_count[1]++;
  else if (sequence == 0b0010 || sequence == 0b1011 || sequence == 0b1101 || sequence == 0b0100)
    left_motor.encoder_count[1]--;
  return;
  */
  if (digitalRead(left_motor.ENCODER_B) == HIGH){
    if (digitalRead(left_motor.ENCODER_A) == HIGH)
      left_motor.encoder_count[1]--;
    else
      left_motor.encoder_count[1]++;
  }
  else{
    if (digitalRead(left_motor.ENCODER_A) == HIGH)
      left_motor.encoder_count[1]++;
    else
      left_motor.encoder_count[1]--;
  }
}

void readRightEncoderA(){
  static unsigned long last_time = 0;
  unsigned long now = micros();
  if (now - last_time < 200) return; // debounce 200us
  last_time = now;

  /*
  right_motor.encoder_count[1] += (digitalRead(right_motor.ENCODER_A) || digitalRead(right_motor.ENCODER_B));
  right_motor.encoder_count[1] -= (digitalRead(right_motor.ENCODER_A) && digitalRead(right_motor.ENCODER_B));
  */
  /*
  right_motor.phaseA[0] = right_motor.phaseA[1];
  right_motor.phaseB[0] = right_motor.phaseB[1];
  right_motor.phaseA[1] = digitalRead(right_motor.ENCODER_A);
  right_motor.phaseB[1] = digitalRead(right_motor.ENCODER_B);

  int encoded = (right_motor.phaseA[1] << 1) | right_motor.phaseB[1];
  int sequence = (right_motor.phaseA[0] << 2) | encoded;
  //int current_pair = (phaseA << 1) + phaseB;
  //int past_pair = (right_phaseA << 1) + right_phaseB;
  //int sequence = (past_pair << 2) + current_pair;

  if (sequence == 0b0001 || sequence == 0b0111 || sequence == 0b1110 || sequence == 0b1000)
    right_motor.encoder_count[1]--;
  else if (sequence == 0b0010 || sequence == 0b1011 || sequence == 0b1101 || sequence == 0b0100)
    right_motor.encoder_count[1]++;
  return;
  */
  if (digitalRead(right_motor.ENCODER_A) == HIGH){
    if (digitalRead(right_motor.ENCODER_B) == HIGH)
      right_motor.encoder_count[1]--;
    else
      right_motor.encoder_count[1]++;
  }
  else{
    if (digitalRead(right_motor.ENCODER_B) == HIGH)
      right_motor.encoder_count[1]++;
    else
      right_motor.encoder_count[1]--;
  }
  return;
}

void readRightEncoderB(){
  /*
  right_motor.encoder_count[1] += (digitalRead(right_motor.ENCODER_A) || digitalRead(right_motor.ENCODER_B));
  right_motor.encoder_count[1] -= (digitalRead(right_motor.ENCODER_A) && digitalRead(right_motor.ENCODER_B));
  */
  /*
  right_motor.phaseA[0] = right_motor.phaseA[1];
  right_motor.phaseB[0] = right_motor.phaseB[1];
  right_motor.phaseA[1] = digitalRead(right_motor.ENCODER_A);
  right_motor.phaseB[1] = digitalRead(right_motor.ENCODER_B);

  int encoded = (right_motor.phaseA[1] << 1) | right_motor.phaseB[1];
  int sequence = (right_motor.phaseA[0] << 2) | encoded;
  //int current_pair = (phaseA << 1) + phaseB;
  //int past_pair = (right_phaseA << 1) + right_phaseB;
  //int sequence = (past_pair << 2) + current_pair;

  if (sequence == 0b0001 || sequence == 0b0111 || sequence == 0b1110 || sequence == 0b1000)
    right_motor.encoder_count[1]--;
  else if (sequence == 0b0010 || sequence == 0b1011 || sequence == 0b1101 || sequence == 0b0100)
    right_motor.encoder_count[1]++;

  if (abs(right_motor.encoder_count[1]) > PPR)
    right_motor.encoder_count[1]
  return;
  */
  if (digitalRead(right_motor.ENCODER_B) == HIGH){
    if (digitalRead(right_motor.ENCODER_A) == HIGH)
      right_motor.encoder_count[1]++;
    else
      right_motor.encoder_count[1]--;
  }
  else{
    if (digitalRead(right_motor.ENCODER_A) == HIGH)
      right_motor.encoder_count[1]--;
    else
      right_motor.encoder_count[1]++;
  }
  return;
}