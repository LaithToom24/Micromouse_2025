#include <math.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>

#ifndef SOLVER_H
#define SOLVER_H

typedef enum Heading {NORTH, EAST, SOUTH, WEST} Heading;
typedef enum Action {LEFT, FORWARD, RIGHT, IDLE} Action;

typedef struct {
    int Coordinate[2];
    int Distance;
    bool walls[4]; // wall[0] - North , wall[1] - East , wall[2] - South , wall[3] - West
    bool Filled;
} Cell;

Action solver();
Action floodFill();

void init_grid();
void serviceQueue();
void recalculateFloodfill();
void shiftQueueUp();
void print_distances();
void print_queue();
void reset_cells(bool);
bool detectWalls();

#endif