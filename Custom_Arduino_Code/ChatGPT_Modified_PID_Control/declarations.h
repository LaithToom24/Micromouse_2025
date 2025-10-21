/* VARIABLES */

int sampling_time = 10000; // in microseconds (us)
int velocity_window = 50000; // in microseconds (us)
float wheel_circumference = 3.14 * 3; // in centimeters (cm)
float current_time, last_time_ctrl, last_time_vel;

int right_phaseA, right_phaseB, left_phaseA, left_phaseB = 0;

// Data definition: Motor
typedef struct{
  // Pin Definitons
  unsigned short int PWM; // Pin to ouput PWM voltage to motor
  unsigned short int DIR; // Pin to set rotation of motor (CW or CCW direction)
  unsigned short int ENCODER_A; // Pin to recieve phase A of encoder input
  unsigned short int ENCODER_B; // Pin to recieve phase B of encoder input
  // RPM[0] = previous RPM 
  // RPM[1] = current RPM
  float RPM[2] = {0};
  // velocity[0] = previous velocity 
  // velocity[1] = current velocity
  float velocity[2] = {0};
  // filtered velocity
  float filtered_velocity[2] = {};
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
  // determines motor orientation (if motor is on left or right side)
  bool LEFT;
  // true if control signal reaches or surpasses threshold
  bool saturated;
  // number of encoder counts per revolution according to specifications
  int specified_CPR;
  // gear ratio of motor
  float gear_ratio;
  // effective CPR for motor
  int CPR;
  // effective PPR for motor
  int PPR;
  // encoder_count[0] = previous encoder count
  // encoder_count[1] = current encoder count
  // tracks the number of pulses emitted by encoder
  // can indicate direction and speed
  volatile long int encoder_count[2] = {0};
  // phase states
  volatile int phaseA[2] = {0};
  volatile int phaseB[2] = {0};
  // PID control parameters
  float Kp;
  float Ki;
  float Kd;
} Motor;

Motor left_motor;
Motor right_motor;

/* FUNCTIONS */

// Individual motor functions
void init_motor(Motor* motor, unsigned short int PWM, unsigned short int DIR, unsigned short int ENCODER_A, unsigned short int ENCODER_B, int specified_CPR, float gear_ratio); // Initiliaze a motor struct
void set_PID_coeffs(Motor* motor, float Kp, float Ki, float Kd);
void compute_speed(Motor* motor); // Use measured encoder counts recieved from motor to estimate motor speed
void compute_control(Motor* motor, float target); // Set motor to target speed using feedback PID control
void set_speed(Motor motor); // Set motor speed according to computed control sinal

// Paired motor functions
void compute_speeds(); // Use measured encoder counts to estimate motor speeds. Also updates derivative term of PID controller.
void compute_controls(float left_target, float right_target); // The PID controller. Does not update derivative term; that is handled by compute_speeds()
void set_speeds(); // Set respective motor speeds according to computed respective control signal
void print_speeds(long int time); // Used for serial logging and plotting measured speeds
void print_RPMs(long int time); // Used for serial logging and plotting measured RPMs
void print_controls(long int time); // Used for serial logging and plotting computed controls

// ISR
void readLeftEncoderA();
void readLeftEncoderB();
void readRightEncoderA();
void readRightEncoderB();