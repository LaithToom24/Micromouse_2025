#include "functions.h"

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);

  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), readLeftEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), readRightEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_B), readLeftEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_B), readRightEncoderB, CHANGE);

  last_time = micros()/1000.0;

  //Serial.println("Starting test!");
}

void loop() {
  float now = micros()/1000.0;
  float time_passed = now - last_time;
  if (time_passed >= sampling_time){
    int left_count_change, right_count_change;
    noInterrupts();
      left_count_change = left_count - left_count_prev;
      right_count_change = right_count - right_count_prev;
      left_count_prev = left_count;
      right_count_prev = right_count;
    interrupts();

    compute_left_speed(left_count_change, time_passed);
    compute_right_speed(right_count_change, time_passed);

    //print_left_speed();
    //print_right_speed();

    last_time = now;

    print_speeds(last_time);

    if (last_time < 5000){
      set_left_speed(3, time_passed);  
      set_right_speed(3, time_passed); 
    }
    else if (5000 <= last_time && last_time < 10000){
      set_left_speed(-7, time_passed);
      set_right_speed(-7, time_passed);
    }
    else{
      set_left_speed(0, time_passed);
      set_right_speed(0, time_passed);
    }

    if (last_time > 16000){
      Serial.end();
    }
  }
}

