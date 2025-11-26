// PID definition
class PID_Controller
{
  public:

  PID_Controller();
  PID_Controller(float Kp, float Ki, float Kd, int control_sampling_time, int variable_sampling_time, float cutoff_freq, float max_control);
  void setup(float Kp, float Ki, float Kd, int control_sampling_time, int variable_sampling_time, float cutoff_freq, float max_control);
  // Create a PID_Controller with values for proportional gain, integral gain, derivative gain, sampling period of process variable, 
  // and cutoff frequency for low-pass filter on derivative term.
  float process(float error, float measured, unsigned long now);
  float process(unsigned long now);
  // Gives the controller the error and outputs the appropriate control. 
  int get_control_loops();
  void clear();

  private:

  float kp, ki, kd;
  float fs, fc;
  float a_d, b_d;
  float a_i;
  int ctrl_samp_time, var_samp_time;
  float control = 0;
  float max_control;
  float error[2] = {0};
  float measured[2] = {0};
  float integral[2] = {0};
  float derivative[2] = {0};
  int control_loops = 0;
  unsigned long last_time = 0;
};