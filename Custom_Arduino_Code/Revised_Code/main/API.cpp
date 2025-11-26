#include "API.hpp"
#include "commands.hpp"

int API_wallFront() {
    return 0;
}

int API_wallRight() {
    return 0;
}

int API_wallLeft() {
    return 1;
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