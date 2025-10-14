const int ENCODER_LEFT_A = 2;
const int ENCODER_RIGHT_A = 3;

const int ENCODER_LEFT_B = 4;
const int ENCODER_RIGHT_B = 5;

const int MOTOR_LEFT_DIR = 7;
const int MOTOR_RIGHT_DIR = 8;

const int MOTOR_LEFT_PWM = 9;
const int MOTOR_RIGHT_PWM = 10;

float total_rotations_right = 0;
float total_rotations_left = 0;
float unfiltered_left_vel = 0;
float unfiltered_right_vel = 0;
float unfiltered_left_vel_prev = 0;
float unfiltered_right_vel_prev = 0;
float left_velocity = 0;
float right_velocity = 0;

float control_signal_left = 0;
float control_signal_left_prev = 0;
float unfiltered_control_signal_left = 0;
float unfiltered_control_signal_left_prev = 0;

float control_signal_right = 0;
float control_signal_right_prev = 0;
float unfiltered_control_signal_right = 0;
float unfiltered_control_signal_right_prev = 0;

float left_velocity_prev = 0;
float right_velocity_prev = 0;
volatile int left_encoderCount = 0;
volatile int right_encoderCount = 0;
volatile int left_phaseA = 0;
volatile int left_phaseB = 0;
volatile int right_phaseA = 0;
volatile int right_phaseB = 0;

float wheel_circumference = 3.0 * 3.14; // in cm
int counts_per_rev = 12;
int gear_ratio = 10;
int sampling_time = 20; // in ms 
long last_time;
float net_left_error = 0;
float last_left_error = 0;
float net_right_error = 0;
float last_right_error = 0;

float kp = 2.6;
float ki = 17.25;
float kd = 0.025;

bool end_test = false;

void set_left_speed(float speed, float time){
  float error = speed - left_velocity;

  float integral = net_left_error + (error * time/1000.0);
  float derivative = 1000 * (error - last_left_error) / time;

  net_left_error = integral;
  last_left_error = error;

  control_signal_left_prev = control_signal_left;
  unfiltered_control_signal_left_prev = unfiltered_control_signal_left;
  unfiltered_control_signal_left = kp * error + ki * integral + kd * derivative;

  //control_signal_left = control_signal_left_prev * 0.3891 + 0.3055 * (unfiltered_control_signal_left + unfiltered_control_signal_left_prev);
  control_signal_left = control_signal_left_prev * 0.4524 + 0.2738 * (unfiltered_control_signal_left + unfiltered_control_signal_left_prev);
  int pwm;
  if (fabs(control_signal_left) > 240)
    pwm = 240;
  else
    pwm = fabs(control_signal_left);
  

  analogWrite(MOTOR_LEFT_PWM, pwm);
  digitalWrite(MOTOR_LEFT_DIR, (-speed > 0));
}

void set_right_speed(float speed, float time){
  float error = speed - right_velocity;

  float integral = net_right_error + (error * time/1000.0);
  float derivative = 1000 * (error - last_right_error) / time;

  net_right_error = integral;
  last_right_error = error;

  control_signal_right_prev = control_signal_right;
  unfiltered_control_signal_right_prev = unfiltered_control_signal_right;
  unfiltered_control_signal_right = kp * error + ki * integral + kd * derivative;

  //control_signal_right = control_signal_right_prev * 0.3891 + 0.3055 * (unfiltered_control_signal_right + unfiltered_control_signal_right_prev);
  control_signal_right = control_signal_right_prev * 0.4524 + 0.2738 * (unfiltered_control_signal_right + unfiltered_control_signal_right_prev);
  int pwm;
  if (fabs(control_signal_right) > 240)
    pwm = 240;
  else
    pwm = fabs(control_signal_right);
  

  analogWrite(MOTOR_RIGHT_PWM, pwm);
  digitalWrite(MOTOR_RIGHT_DIR, (speed > 0));
}

void compute_right_speed(int right_count, int time){
  right_velocity_prev = right_velocity;
  unfiltered_right_vel_prev =  1000 * total_rotations_right * wheel_circumference / time; 
  total_rotations_right = right_count / ((float)counts_per_rev * gear_ratio);
  unfiltered_right_vel = 1000 * total_rotations_right * wheel_circumference / time; 

  right_velocity = -0.222 * right_velocity_prev + 0.6109 * (unfiltered_right_vel + unfiltered_right_vel_prev);
}

void compute_left_speed(int left_count, int time){
  left_velocity_prev = left_velocity;
  unfiltered_left_vel_prev =  1000 * total_rotations_left * wheel_circumference / time; 
  total_rotations_left = left_count / ((float)counts_per_rev * gear_ratio);
  unfiltered_left_vel = 1000 * total_rotations_left * wheel_circumference / time; 

  left_velocity = -0.222 * left_velocity_prev + 0.6109 * (unfiltered_left_vel + unfiltered_left_vel_prev);
}

void print_left_speed(){
  Serial.print("Left speed: ");
  Serial.print(left_velocity, 4);
  Serial.println(" cm/s");
}

void print_right_speed(){
  Serial.print("Right speed: ");
  Serial.print(right_velocity, 4);
  Serial.println(" cm/s");
}

void print_speeds(unsigned long t) {
  Serial.print(t);
  Serial.print(",");
  Serial.print(left_velocity, 4);
  Serial.print(",");
  Serial.println(right_velocity, 4);
}

// ISR Functions
void readLeftEncoder(){
  int phaseA = digitalRead(ENCODER_LEFT_A);
  int phaseB = digitalRead(ENCODER_LEFT_B);

  int encoded = (phaseA << 1) | phaseB;
  int sequence = (left_phaseA << 2) | encoded;

  if (sequence == 0b0001 || sequence == 0b0111 || sequence == 0b1110 || sequence == 0b1000){
    left_encoderCount--;
  }
  else if (sequence == 0b0010 || sequence == 0b1011 || sequence == 0b1101 || sequence == 0b0100){
    left_encoderCount++;
  }

  left_phaseA = phaseA;
  left_phaseB = phaseB;
}

void readRightEncoder(){
  int phaseA = digitalRead(ENCODER_RIGHT_A);
  int phaseB = digitalRead(ENCODER_RIGHT_B);

  int encoded = (phaseA << 1) | phaseB;
  int sequence = (right_phaseA << 2) | encoded;

  if (sequence == 0b0001 || sequence == 0b0111 || sequence == 0b1110 || sequence == 0b1000){
    right_encoderCount--;
  }
  else if (sequence == 0b0010 || sequence == 0b1011 || sequence == 0b1101 || sequence == 0b0100){
    right_encoderCount++;
  }

  right_phaseA = phaseA;
  right_phaseB = phaseB;
}