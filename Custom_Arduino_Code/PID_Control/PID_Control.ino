#include "functions.h"
#include <util/atomic.h>

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

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), readLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), readRightEncoder, CHANGE);

  last_time = millis();

  //Serial.println("Starting test!");
}

void loop() {
  float time_passed = millis() - last_time;
  if (time_passed >= sampling_time && !end_test){
    int left_count, right_count;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      left_count = left_encoderCount;
      right_count = right_encoderCount;
      left_encoderCount = 0;
      right_encoderCount = 0;
    }

    compute_left_speed(left_count, time_passed);
    compute_right_speed(right_count, time_passed);

    //print_left_speed();
    //print_right_speed();

    last_time = millis();

    print_speeds(last_time);

    if (last_time < 5000){
      set_left_speed(100, time_passed);   // left wheel forward at 50 cm/s
      set_right_speed(100, time_passed); // right wheel backward at about 40% power
    }
    else if (5000 <= last_time && last_time < 10000){
      set_left_speed(50, time_passed);
      set_right_speed(50, time_passed);
    }
    else{
      set_left_speed(0, time_passed);
      set_right_speed(0, time_passed);
    }

    if (last_time > 15000){
      end_test = true;
    }
  }

  if (end_test){
    Serial.end();
    exit(0);
  }
}

