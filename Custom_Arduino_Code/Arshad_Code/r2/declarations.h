#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// ==== Infrared Sensor Hardware ====
#define EMITTER 12   // LED emitter pin
#define SENSOR_RIGHT_MARK A0
#define SENSOR_1 A1
#define SENSOR_2 A2
#define SENSOR_3 A3
#define SENSOR_4 A4
#define SENSOR_LEFT_MARK A5
#define FUNCTION_PIN A6
#define BATTERY_VOLTS A7
#define LEFT_WALL_LED 6

extern int sensorDark;
extern int sensorLit;
extern int sensorValue;

// Sensor functions
void init_sensors();
int  readSensor(int channel);
int readFront();
int readRight();
int readRight();
void readAll();
float wall_pid(float base_speed, float left_dist, float dt);


/* VARIABLES */
extern int sampling_time; // in microseconds (us)
extern int control_loops = 0; // number of control loops (reset to zero after reaching velocity_window)
extern int velocity_window = 10; // in number of control loops
float wheel_circumference = M_PI * 3; // in centimeters (cm)
extern long int current_time, last_time;
int debounce_time = 5; // in us
float left_target, right_target;
bool is_turning = false;
bool done_turning = true;
bool done_straight = true;
bool done_follow = true;
extern int front_dist;
extern int right_dist;
extern int left_dist;
extern bool wall_front;
extern bool wall_left;
extern bool wall_right;
extern int diffF;

extern float wall_Kp;
extern float wall_Ki;
extern float wall_Kd;
extern float wall_integral;
extern float wall_derivative;
extern float wall_prev_error;
extern unsigned long last_wall_time;



// Data definition: Motor
typedef struct{
  // Pin Definitons
  unsigned short int PWM; // Pin to ouput PWM voltage to motor
  unsigned short int DIR; // Pin to set rotation of motor (CW or CCW direction)
  unsigned short int ENCODER_AxorB; // Pin to recieve phase A of encoder input
  unsigned short int ENCODER_B; // Pin to recieve phase B of encoder input
  // RPM[0] = previous RPM 
  // RPM[1] = current RPM
  float RPM[2] = {0};
  // velocity[0] = previous velocity 
  // velocity[1] = current velocity
  float velocity[2] = {0};
  // filtered velocity
  float filtered_velocity[2] = {0};
  // integral[0] = past integral
  // integral[1] = current integral
  float integral[2] = {0};
  // derivative[0] = past derivative
  // derivative[1] = current derivative
  float derivative[2] = {0};
  float filtered_derivative[2] = {0};
  // error[0] = past_error
  // error[1] = current_error
  // the difference between our target speed and estimated actual speed
  float error[2] = {0};
  // control[0] = previous control signal 
  // control[1] = current control signal
  // signed PWM signal computed by PID controller using feedback
  // the control signal consists of a direction (+ or -) and magnitude (PWM value between 0 and 240)
  float control[2] = {0};
  // the filtered control signal consists of a direction (+ or -) and magnitude (PWM value between 0 and 240)
  float filtered_control[2] = {0};
  // determines motor orientation (if motor is on left or right side)
  bool LEFT;
  // true if control signal reaches or surpasses threshold
  bool saturated;
  // trye if integrator reaches or surpasses threshold
  bool integrator_saturated;
  // number of encoder counts per revolution according to specifications
  int specified_CPR;
  // gear ratio of motor
  float gear_ratio;
  // effective CPR for motor
  float CPR;
  // effective PPR for motor
  float PPR;
  // encoder_count[0] = previous encoder count
  // encoder_count[1] = current encoder count
  // tracks the number of pulses emitted by encoder
  // can indicate direction and speed
  volatile long int encoder_count = 0;
  // phase states
  volatile int phaseA[2] = {0};
  volatile int phaseB[2] = {0};
  // PID control parameters
  float Kp;
  float Ki;
  float Kd;
  // coefficients for low-pass filter of derivative
  float b_d;
  float a_d;
  // coefficients for low-pass filter of control
  float b_c;
  float a_c;
  // encoder count for rotation
  volatile int rotational_encoder_count = 0;
} Motor;

Motor left_motor;
Motor right_motor;

/* FUNCTIONS */

// Individual motor functions
void init_motor(Motor* motor, unsigned short int PWM, unsigned short int DIR, unsigned short int ENCODER_AxorB, unsigned short int ENCODER_B, int specified_CPR, float gear_ratio); // Initiliaze a motor struct
void set_PID_coeffs(Motor* motor, float Kp, float Ki, float Kd);
void compute_speed(Motor* motor); // Use measured encoder counts recieved from motor to estimate motor speed
void compute_control(Motor* motor, float target); // Set motor to target speed using feedback PID control
void set_speeds(Motor motor); // Set motor speed according to computed control sinal
void compute_lowpass_filter_coeffs(Motor* motor, int sampling_time, float cutoff_freq);

// Paired motor functions
void compute_speeds(); // Use measured encoder counts to estimate motor speeds. Also updates derivative term of PID controller.
void compute_controls(float left_target, float right_target); // The PID controller. Does not update derivative term; that is handled by compute_speeds()
void send_controls(); // Set respective motor speeds according to computed respective control signal
void print_speeds(long int time); // Used for serial logging and plotting measured speeds
void print_RPMs(long int time); // Used for serial logging and plotting measured RPMs
void print_controls(long int time); // Used for serial logging and plotting computed controls
void print_speeds_and_controls(long int time); // Used for serial logging and plotting estimated speeds and computed controls
void set_targets(float left, float right);
bool turn(float degs);
bool straight(int dir, float dist);
bool LeftWallPID(int dir, float dist);


// ISR
void readLeftEncoderA();
void readLeftEncoderB();
void readRightEncoderA();
void readRightEncoderB();

// IR SENSOR
void init_sensors();
int readSensor(int channel);
int readFront();
int readRight();
int readLeft();
void detectWalls();
void LeftWallFollower();
void LeftWallPID(float base_speed, float target_dist);
