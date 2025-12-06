#include <Arduino.h>
#include "API.hpp"
#include "commands.hpp"
#include "sensor.hpp"

int API_wallFront() {
    return front_dist >= 200;
}

int API_wallRight() {
    return right_dist >= 200;
}

int API_wallLeft() {
    return left_dist >= 200;
}

void API_moveForward(){
    add_command(0, 7);
}

void API_turnLeft(){
    add_command(1, -90);
}

void API_turnRight(){
    add_command(1, 90);
}