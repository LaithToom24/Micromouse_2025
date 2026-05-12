#ifndef SENSOR_HPP
#define SENSOR_HPP

extern int left_dist, right_dist, front_dist;

int readSensor(int channel);
void readAll();
void printAll();

#endif