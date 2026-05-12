#include <math.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>

#ifndef SOLVER_H
#define SOLVER_H

typedef enum Heading {NORTH, EAST, SOUTH, WEST} Heading;
typedef enum Action {LEFT, FORWARD, RIGHT, IDLE} Action;

typedef struct Cell{
    uint8_t Coordinate[2];
    uint16_t Distance;
    bool walls[4]; // wall[0] - North , wall[1] - East , wall[2] - South , wall[3] - West
    bool Filled;
};

Action solver(bool, bool, bool);
Action floodFill(bool, bool, bool);

void init_grid();
void serviceQueue();
void recalculateFloodfill();
void shiftQueueUp();
void reset_cells(bool);
bool setWall();
bool detectWalls(bool, bool, bool);

#endif