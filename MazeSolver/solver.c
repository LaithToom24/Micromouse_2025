#include "solver.h"
#include "API.h"

#define LENGTH 16
#define AREA LENGTH*LENGTH
Cell cells[AREA];
bool cellFilled[AREA];
int grid[LENGTH][LENGTH];
#define MAX_QUEUE 512
Cell queue[MAX_QUEUE+1];
int queueSize = 0;
bool init = true;
bool right = false;
bool left = false;
bool forward = true;
bool alreadyFound = false;
bool wall_front, wall_left, wall_right;
int iterations = 0;
int bot_y_velocity = 1;
int bot_x_velocity = 0;
int bot_x_pos = 0;
int bot_y_pos = 0;
int left_cell_distance;
int right_cell_distance;
int front_cell_distance;
char str[20];

void init_grid(){ 
    reset_cells();
    queue[0] = cells[8 + (7 * LENGTH)];
    queue[1] = cells[8 + (8 * LENGTH)];
    queue[2] = cells[7 + (8 * LENGTH)];
    queue[3] = cells[7 + (7 * LENGTH)];
    queueSize = 4;
    while (queueSize > 0){
        serviceQueue();
    }
    print_grid();
    debug_log("Maze initially filled.");
};

void print_grid(){
    //debug_log("UPDATING MANHATTAN DISTANCES");
    for (int i = 0; i < LENGTH; i++){
        for (int j = 0; j < LENGTH; j++){
            sprintf(str, "%d", grid[i][j]);
            API_setText(i, j, str);
        }
    }
}

void print_queue(){
    debug_log("PRINTING QUEUE");
    for (int i = 0; i < queueSize; i++){
        sprintf(str, "x: %d, y: %d, distance: %d", queue[i].Coordinate[0], queue[i].Coordinate[1], queue[i].Distance);
        debug_log(str);
    }
}

void serviceQueue(){
    int x = queue[queueSize-1].Coordinate[0];
    int y = queue[queueSize-1].Coordinate[1];
    int distance = queue[queueSize-1].Distance;

    // sprintf(coordinate_string, "x: %d, y: %d, distance: %d", x, y, distance);
    // debug_log(coordinate_string);

    
    if (queueSize > 1 && queue[queueSize-2].Coordinate[0] == x && queue[queueSize-2].Coordinate[1] == y){
        queueSize--;
        return;
    }

    if (x + 1 < LENGTH && (!cells[x + 1 + LENGTH * y].walls[3] && !cells[x + LENGTH * y].walls[3]) && !cellFilled[(x + 1) + y * LENGTH]){
        grid[y][x + 1] = distance + 1; 
        cells[(x + 1) + y * LENGTH].Distance = distance + 1;
        cellFilled[(x + 1) + y * LENGTH] = true;
        shiftQueueUp();
        queue[0] = cells[(x + 1) + y * LENGTH];
    }
    if (x - 1 > -1 && (!cells[x - 1 + LENGTH * y].walls[1] && !cells[x + LENGTH * y].walls[1]) && !cellFilled[(x - 1) + y * LENGTH]){
        grid[y][x - 1] = distance + 1;
        cells[(x - 1) + y * LENGTH].Distance = distance + 1;
        cellFilled[(x - 1) + y * LENGTH] = true;
        shiftQueueUp();
        queue[0] = cells[(x - 1) + y * LENGTH];
    }

    if (y + 1 < LENGTH && (!cells[x + LENGTH * (y + 1)].walls[2] && !cells[x + LENGTH * y].walls[2]) && !cellFilled[x + (y + 1) * LENGTH]){
        grid[y + 1][x] = distance + 1; 
        cells[x + (y + 1) * LENGTH].Distance = distance + 1;
        cellFilled[x + (y + 1) * LENGTH] = true;
        shiftQueueUp();
        queue[0] = cells[x + (y + 1) * LENGTH];
    }
    if (y - 1 > -1 && (!cells[x + LENGTH * (y - 1)].walls[0] && !cells[x + LENGTH * y].walls[0]) && !cellFilled[x + (y - 1) * LENGTH]){
        grid[y - 1][x] = distance + 1;
        cells[x + (y - 1) * LENGTH].Distance = distance + 1;
        cellFilled[x + (y - 1) * LENGTH] = true;
        shiftQueueUp();
        queue[0] = cells[x + (y - 1) * LENGTH];
    }

    queueSize--;
}

void recalculateFloodfill(){
    reset_cells(); 
    queue[0] = cells[8 + (7 * LENGTH)];
    queue[1] = cells[8 + (8 * LENGTH)];
    queue[2] = cells[7 + (8 * LENGTH)];
    queue[3] = cells[7 + (7 * LENGTH)];
    queueSize = 4;
    while (queueSize > 0){
        serviceQueue();
    }
    print_grid();
}

void shiftQueueUp(){
    for (int i = queueSize-1; i >= 0; i--){
        Cell temp = queue[i];
        queue[i] = queue[i+1];
        queue[i+1] = temp;
    }
    queueSize++;
}

void reset_cells(){
    for (int i = 0; i < LENGTH; i++){
        for (int j = 0; j < LENGTH; j++){
            cells[j+LENGTH*i].Distance = grid[i][j] + 1;
            cells[j+LENGTH*i].Coordinate[0] = j;
            cells[j+LENGTH*i].Coordinate[1] = i;
            cellFilled[j + i * LENGTH] = false;
            if (init){
                cells[j+LENGTH*i].walls[0] = false;
                cells[j+LENGTH*i].walls[1] = false;
                cells[j+LENGTH*i].walls[2] = false;
                cells[j+LENGTH*i].walls[3] = false;
            }
        }
    }
    cells[8 + (7 * LENGTH)].Distance = 0;
    cells[8 + (8 * LENGTH)].Distance = 0;
    cells[7 + (8 * LENGTH)].Distance = 0;
    cells[7 + (7 * LENGTH)].Distance = 0;
    cellFilled[8 + (7 * LENGTH)] = true;
    cellFilled[8 + (8 * LENGTH)] = true;
    cellFilled[7 + (8 * LENGTH)] = true;
    cellFilled[7 + (7 * LENGTH)] = true;
}

//void set_cell_distance(int newDistance, int i){
//    cells[i].Distance = newDistance;
//}

Action solver() {
    if (init){
        init_grid();
        init = false;
    } 
    return floodFill();
}

// This is an example of a simple left wall following algorithm.
Action leftWallFollower() {
    if(API_wallFront()) {
        if(API_wallLeft()){
            return RIGHT;
        }
        return LEFT;
    }
    return FORWARD;
}


// Put your implementation of floodfill here!
Action floodFill() {
    Action nextMove = FORWARD; 
    // alreadyFound = false;

    // if (API_wallLeft()){
    //     char wall_direction;
    //     if (bot_x_velocity == 1){
    //         wall_direction = 'n';
    //         cells[bot_x_pos + LENGTH * bot_y_pos].walls[1] = true;
    //         // debug_log("e");
    //     }
    //     else if (bot_x_velocity == -1){
    //         wall_direction = 's';
    //         cells[bot_x_pos + LENGTH * bot_y_pos].walls[3] = true;
    //         // debug_log("w");
    //     }
    //     else if (bot_y_velocity == 1){
    //         wall_direction = 'w';
    //         cells[bot_x_pos + LENGTH * bot_y_pos].walls[0] = true;
    //         // debug_log("n");
    //     }
    //     else if (bot_y_velocity == -1){
    //         wall_direction = 'e';
    //         cells[bot_x_pos + LENGTH * bot_y_pos].walls[2] = true;
    //         // debug_log("s");
    //     }
    //     API_setWall(bot_x_pos, bot_y_pos, wall_direction);
    // }
    // else if (API_wallRight()){
    //     char wall_direction;
    //     if (bot_x_velocity == 1){
    //         wall_direction = 's';
    //         cells[bot_x_pos + LENGTH * bot_y_pos].walls[1] = true;
    //         // debug_log("e");
    //     }
    //     else if (bot_x_velocity == -1){
    //         wall_direction = 'n';
    //         cells[bot_x_pos + LENGTH * bot_y_pos].walls[3] = true;
    //         // debug_log("w");
    //     }
    //     else if (bot_y_velocity == 1){
    //         wall_direction = 'e';
    //         cells[bot_x_pos + LENGTH * bot_y_pos].walls[0] = true;
    //         // debug_log("n");
    //     }
    //     else if (bot_y_velocity == -1){
    //         wall_direction = 'w';
    //         cells[bot_x_pos + LENGTH * bot_y_pos].walls[2] = true;
    //         // debug_log("s");
    //     }
    //     API_setWall(bot_x_pos, bot_y_pos, wall_direction);
    // }
    
    if(API_wallFront()) {
        char wall_direction;
        if (bot_x_velocity == 1){
            wall_direction = 'e';
            cells[bot_x_pos + LENGTH * bot_y_pos].walls[1] = true;
            // debug_log("e");
        }
        else if (bot_x_velocity == -1){
            wall_direction = 'w';
            cells[bot_x_pos + LENGTH * bot_y_pos].walls[3] = true;
            // debug_log("w");
        }
        else if (bot_y_velocity == 1){
            wall_direction = 'n';
            cells[bot_x_pos + LENGTH * bot_y_pos].walls[0] = true;
            // debug_log("n");
        }
        else if (bot_y_velocity == -1){
            wall_direction = 's';
            cells[bot_x_pos + LENGTH * bot_y_pos].walls[2] = true;
            // debug_log("s");
        }
        API_setWall(bot_x_pos, bot_y_pos, wall_direction);
        nextMove = LEFT;
        if(API_wallLeft()){
            if (wall_direction == 'n'){
                wall_direction = 'w';
                cells[bot_x_pos + LENGTH * bot_y_pos].walls[3] = true;
            }
            else if (wall_direction == 'e'){
                wall_direction = 'n';
                cells[bot_x_pos + LENGTH * bot_y_pos].walls[0] = true;
            }
            else if (wall_direction == 's'){
                wall_direction = 'e';
                cells[bot_x_pos + LENGTH * bot_y_pos].walls[1] = true;
            }
            else if (wall_direction == 'w'){
                wall_direction == 's';
                cells[bot_x_pos + LENGTH * bot_y_pos].walls[2] = true;
            }
            API_setWall(bot_x_pos, bot_y_pos, wall_direction);
            nextMove = RIGHT;
        }

        recalculateFloodfill();
    } 
    else{
        if (bot_x_velocity == -1){
            wall_left = cells[bot_x_pos + bot_y_pos * LENGTH].walls[2];
            wall_right = cells[bot_x_pos + bot_y_pos * LENGTH].walls[0];
            wall_front = cells[bot_x_pos + bot_y_pos * LENGTH].walls[3];
        }
        else if (bot_x_velocity == 1){
            wall_left = cells[bot_x_pos + bot_y_pos * LENGTH].walls[0];
            wall_right = cells[bot_x_pos + bot_y_pos * LENGTH].walls[2];
            wall_front = cells[bot_x_pos + bot_y_pos * LENGTH].walls[1];
        }
        else if (bot_y_velocity == -1){
            wall_left = cells[bot_x_pos + bot_y_pos * LENGTH].walls[1];
            wall_right = cells[bot_x_pos + bot_y_pos * LENGTH].walls[3];
            wall_front = cells[bot_x_pos + bot_y_pos * LENGTH].walls[2];
        }
        else{
            wall_left = cells[bot_x_pos + bot_y_pos * LENGTH].walls[3];
            wall_right = cells[bot_x_pos + bot_y_pos * LENGTH].walls[1];
            wall_front = cells[bot_x_pos + bot_y_pos * LENGTH].walls[0];
        }

        sprintf(str, "wall_front: %d, wall_left: %d, wall_right: %d", wall_front, wall_left, wall_right);
        debug_log(str);

        if (bot_x_pos < bot_y_velocity || wall_left)
            left_cell_distance = 1000;
        else
            left_cell_distance = cells[(bot_x_pos - bot_y_velocity) + LENGTH * (bot_y_pos + bot_x_velocity)].Distance;
            sprintf(str, "Left Cell Coordinates are (%d, %d)", bot_x_pos - bot_y_velocity, bot_y_pos - bot_x_velocity);
            debug_log(str);

        if (bot_y_pos < bot_x_velocity || wall_right)
            right_cell_distance = 1000;
        else
            right_cell_distance = cells[(bot_x_pos + bot_y_velocity) + LENGTH * (bot_y_pos - bot_x_velocity)].Distance;

        if (wall_front)
            front_cell_distance = 1000;
        else
            front_cell_distance = cells[(bot_x_pos + bot_x_velocity) + LENGTH * (bot_y_pos + bot_y_velocity)].Distance;

        if (left_cell_distance <= right_cell_distance){
            if (left_cell_distance < front_cell_distance)
                nextMove = LEFT;
            sprintf(str, "front_cell_distance: %d, left_cell_distance: %d", front_cell_distance, left_cell_distance);
            debug_log(str);
        }
        else if (right_cell_distance < left_cell_distance){
            if (right_cell_distance < front_cell_distance)
                nextMove = RIGHT;
        }
    }

    if (nextMove == RIGHT){ 
        debug_log("TURNING RIGHT.");
        if (bot_y_velocity == 1){
            bot_x_velocity = 1;
            bot_y_velocity = 0;
        }
        else if (bot_y_velocity == -1){
            bot_x_velocity = -1;
            bot_y_velocity = 0;
        }
        else{
            if (bot_x_velocity == -1){
                bot_y_velocity = 1;
            }
            else if (bot_x_velocity == 1){
                bot_y_velocity = -1;
            }
            bot_x_velocity = 0;
        }
    }
    else if (nextMove == LEFT){
        debug_log("TURNING LEFT.");
        if (bot_y_velocity == 1){
            bot_x_velocity = -1;
            bot_y_velocity = 0;
        }
        else if (bot_y_velocity == -1){
            bot_x_velocity = 1;
            bot_y_velocity = 0;
        }
        else{
            if (bot_x_velocity == -1){
                bot_y_velocity = -1;
            }
            else if (bot_x_velocity == 1){
                bot_y_velocity = 1;
            }
            bot_x_velocity = 0;
        }
    }
    else{ 
        sprintf(str, "x: %d, y: %d", bot_x_pos, bot_y_pos);
        debug_log(str);
        sprintf(str, "x_vel: %d, y_vel: %d", bot_x_velocity, bot_y_velocity);
        debug_log(str);
        bot_x_pos += bot_x_velocity;
        bot_y_pos += bot_y_velocity;
    } 

    return nextMove;
    
}