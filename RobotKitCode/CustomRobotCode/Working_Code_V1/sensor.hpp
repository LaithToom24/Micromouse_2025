#ifndef SENSOR_HPP
#define SENSOR_HPP

#include "Adafruit_VL53L0X.h"

//VL53L0X_RangingMeasurementData_t left_dist, right_dist, front_dist;

extern Adafruit_VL53L0X lox1;
extern Adafruit_VL53L0X lox2;
extern Adafruit_VL53L0X lox3;

extern VL53L0X_RangingMeasurementData_t left_dist;
extern VL53L0X_RangingMeasurementData_t right_dist;
extern VL53L0X_RangingMeasurementData_t front_dist;

int readSensor(int channel);
void setID();
void ToF_setup();
void ToF_test();

void readAll();
int readLeft();
int readRight();
int readFront();

#endif