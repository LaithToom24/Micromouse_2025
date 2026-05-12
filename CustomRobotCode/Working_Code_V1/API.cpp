#include <Arduino.h>
#include "API.hpp"
#include "commands.hpp"
#include "sensor.hpp"

void API_moveForward(){
    add_command(0, 2.0);
}

void API_turnLeft(){
    add_command(1, -90);
}

void API_turnRight(){
    add_command(1, 90);
}