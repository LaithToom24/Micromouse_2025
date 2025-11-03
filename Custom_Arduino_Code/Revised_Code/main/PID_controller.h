#include <math.h>

// PID definition
class PID_Controller
{
  public:

  PID_Controller();
  PID_Controller(float Kp, float Ki, float Kd, int control_sampling_time, int variable_sampling_time, float cutoff_freq);
  void setup(float Kp, float Ki, float Kd, int control_sampling_time, int variable_sampling_time, float cutoff_freq);
  // Create a PID_Controller with values for proportional gain, integral gain, derivative gain, sampling period of process variable, 
  // and cutoff frequency for low-pass filter on derivative term.
  float process(float error, float measured, unsigned long now);
  float process(unsigned long now);
  // Gives the controller the error and outputs the appropriate control. 
  int get_control_loops();

  private:

  float kp, ki, kd;
  float fs, fc;
  float a_d, b_d;
  float a_i;
  int ctrl_samp_time, var_samp_time;
  float control = 0;
  float error[2] = {0};
  float measured[2] = {0};
  float integral[2] = {0};
  float derivative[2] = {0};
  int control_loops = 0;
  unsigned long last_time = 0;
};

// PID implementation
PID_Controller::PID_Controller(){
  kp = 1;
  ki = 0;
  kd = 0;
  ctrl_samp_time = 10000;
  var_samp_time = 1; // as positive integer multiple (counts of) of control loops
  fc = 5;
  fs = 1e6f/(float)ctrl_samp_time;
  // Coeffs for Difference Equation Implementation of Filtered Differentiator
  a_d = 2 * M_PI * fs * fc / (fs + M_PI * fc);
  b_d = (fs - M_PI * fc) / (fs + M_PI * fc);
  // Coeff for Difference Equation Implementation of Integrator
  a_i = 0.5e-6f * ctrl_samp_time;
}

PID_Controller::PID_Controller(float Kp, float Ki, float Kd, int control_sampling_time, int variable_sampling_time, float cutoff_freq){
  kp = Kp;
  ki = Ki;
  kd = Kd;
  ctrl_samp_time = control_sampling_time;
  var_samp_time = variable_sampling_time; // as positive integer multiple (counts of) of control loops
  fc = cutoff_freq;
  fs = 1e6f/(float)ctrl_samp_time;
  // Coeffs for Difference Equation Implementation of Filtered Differentiator
  a_d = 2.0f * M_PI * fs * fc / (fs + M_PI * fc);
  b_d = (fs - M_PI * fc) / (fs + M_PI * fc);
  // Coeff for Difference Equation Implementation of Integrator
  a_i = 0.5e-6f * ctrl_samp_time;
}

void PID_Controller::setup(float Kp, float Ki, float Kd, int control_sampling_time, int variable_sampling_time, float cutoff_freq){
  kp = Kp;
  ki = Ki;
  kd = Kd;
  ctrl_samp_time = control_sampling_time;
  var_samp_time = variable_sampling_time; // as positive integer multiple (counts of) of control loops
  fc = cutoff_freq;
  fs = 1e6f/(float)ctrl_samp_time;
  // Coeffs for Difference Equation Implementation of Filtered Differentiator
  a_d = 2 * M_PI * fs * fc / (fs + M_PI * fc);
  b_d = (fs - M_PI * fc) / (fs + M_PI * fc);
  // Coeff for Difference Equation Implementation of Integrator
  a_i = 0.5 * 1e-6f * ctrl_samp_time;
}

float PID_Controller::process(float newError, float newMeasure, unsigned long now){
  if (now - last_time >= ctrl_samp_time){
    error[0] = error[1];
    error[1] = newError;

    // Bilinear Transform of Low-Pass Filtered Differentiator
    derivative[0] = derivative[1];
    if (control_loops == var_samp_time){
      measured[1] = newMeasure; 
      derivative[1] = b_d * derivative[0] + a_d * (measured[1] - measured[0]);
      measured[0] = measured[1];
      control_loops = 0;
    }

    // PD portion of Control
    float pd = kp * error[1] + kd * derivative[1];

    integral[0] = integral[1];
    // Bilinear Transform of Integrator
    integral[1] = integral[0] + a_i * (error[0] + error[1]); 
    // Anti-Windup by Saturating Integral with bounds [-PD/Ki, +PD/Ki]
    if (ki != 0) 
      integral[1] = constrain(integral[1], -255.0f/ki, 255.0f/ki);

    // Control signal
    control = ki * integral[1] + pd;

    control = constrain(control, -255.0f, 255.0f); 

    //Serial.println(pd);

    control_loops++;
    last_time = now;
  }

  return control;
}

float PID_Controller::process(unsigned long now){
  if (now - last_time >= ctrl_samp_time){

    if (control_loops == var_samp_time)
      control_loops = 0;

    control_loops++;
    last_time = now;
  }

  return 0.0f;
}

int PID_Controller::get_control_loops(){
  return control_loops;
}