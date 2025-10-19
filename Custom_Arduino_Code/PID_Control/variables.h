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

volatile int left_count = 0;
volatile int right_count = 0;
int left_count_prev = 0;
int right_count_prev = 0;
volatile int left_phaseA = 0;
volatile int left_phaseB = 0;
volatile int right_phaseA = 0;
volatile int right_phaseB = 0;

float wheel_circumference = 3.0 * 3.14; // in cm
int counts_per_rev = 12 * 130/3;
int gear_ratio = 10;
int sampling_time = 20; // in ms 
long last_time;
float net_left_error = 0;
float last_left_error = 0;
float net_right_error = 0;
float last_right_error = 0;

float kp = 35;
float ki = 220;
float kd = 1.15;

bool end_test = false;