#include "solver.h"
#include "API.h"

#define LENGTH 16
#define AREA LENGTH*LENGTH
Cell cells[AREA];
#define MAX_QUEUE 512
Cell queue[MAX_QUEUE+1];
int queueSize = 0;
bool init_phase = true;
bool wall_front, wall_left, wall_right;
int bot_y_velocity = 1;
int bot_x_velocity = 0;
int bot_x_pos = 0;
int bot_y_pos = 0;
int goal_cells_found = 0;
int goal_cells_existing = 4;
bool goal_reached = false;
bool print_goal_message = true;
char str[15];

void init_grid(){ 
    reset_cells(false);
    queue[0] = cells[8 + (7 * LENGTH)];
    queue[1] = cells[8 + (8 * LENGTH)];
    queue[2] = cells[7 + (8 * LENGTH)];
    queue[3] = cells[7 + (7 * LENGTH)];
    queueSize = 4;
    while (queueSize > 0){
        serviceQueue();
    }
}

void shiftQueueUp() {
    for (int i = queueSize - 1; i >= 0; i--) {
        Cell temp = queue[i];
        queue[i] = queue[i + 1];
        queue[i + 1] = temp;
    }
    queueSize++;
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

    if (x + 1 < LENGTH && (!cells[x + 1 + LENGTH * y].walls[3] && !cells[x + LENGTH * y].walls[1]) && !cells[(x + 1) + y * LENGTH].Filled) {
        cells[(x + 1) + y * LENGTH].Distance = distance + 1;
        cells[(x + 1) + y * LENGTH].Filled = true;
        shiftQueueUp();
        queue[0] = cells[(x + 1) + y * LENGTH];
    }
    if (x - 1 > -1 && (!cells[x - 1 + LENGTH * y].walls[1] && !cells[x + LENGTH * y].walls[3]) && !cells[(x - 1) + y * LENGTH].Filled) {
        cells[(x - 1) + y * LENGTH].Distance = distance + 1;
        cells[(x - 1) + y * LENGTH].Filled = true;
        shiftQueueUp();
        queue[0] = cells[(x - 1) + y * LENGTH];
    }
    if (y + 1 < LENGTH && (!cells[x + LENGTH * (y + 1)].walls[2] && !cells[x + LENGTH * y].walls[0]) && !cells[x + (y + 1) * LENGTH].Filled) {
        cells[x + (y + 1) * LENGTH].Distance = distance + 1;
        cells[x + (y + 1) * LENGTH].Filled = true;
        shiftQueueUp();
        queue[0] = cells[x + (y + 1) * LENGTH];
    }
    if (y - 1 > -1 && (!cells[x + LENGTH * (y - 1)].walls[0] && !cells[x + LENGTH * y].walls[2]) && !cells[x + (y - 1) * LENGTH].Filled) {
        cells[x + (y - 1) * LENGTH].Distance = distance + 1;
        cells[x + (y - 1) * LENGTH].Filled = true;
        shiftQueueUp();
        queue[0] = cells[x + (y - 1) * LENGTH];
    }

    queueSize--;
}

void recalculateFloodfill(){
    // clear cells for recalculation
    reset_cells(goal_cells_existing != 4);

    // add goal cells to queue
    if (goal_cells_existing == 4) {
        queue[0] = cells[8 + (7 * LENGTH)];
        queue[1] = cells[8 + (8 * LENGTH)];
        queue[2] = cells[7 + (8 * LENGTH)];
        queue[3] = cells[7 + (7 * LENGTH)];
        queueSize = 4;
    }
    else {
        queue[0] = cells[0];
        queueSize = 1;
    }

    // perform recalculation
    while (queueSize > 0){
        serviceQueue();
    }
}

void reset_cells(bool goal_is_start){
    // reset cells (keep wall information intact if not in initialization phase)
    for (int i = 0; i < LENGTH; i++){
        for (int j = 0; j < LENGTH; j++){
            cells[j+LENGTH*i].Distance = 1;
            cells[j+LENGTH*i].Coordinate[0] = j;
            cells[j+LENGTH*i].Coordinate[1] = i;
            cells[j + i * LENGTH].Filled = false;
            if (init_phase){
                cells[j+LENGTH*i].walls[0] = false;
                cells[j+LENGTH*i].walls[1] = false;
                cells[j+LENGTH*i].walls[2] = false;
                cells[j+LENGTH*i].walls[3] = false;
            }
        }
    }

    // set goal cells (where the goal is on the map)
    if (!goal_is_start) {
        cells[8 + (7 * LENGTH)].Distance = 0;
        cells[8 + (8 * LENGTH)].Distance = 0;
        cells[7 + (8 * LENGTH)].Distance = 0;
        cells[7 + (7 * LENGTH)].Distance = 0;
        cells[8 + (7 * LENGTH)].Filled = true;
        cells[8 + (8 * LENGTH)].Filled = true;
        cells[7 + (8 * LENGTH)].Filled = true;
        cells[7 + (7 * LENGTH)].Filled = true;
    }
    else {
        cells[0].Distance = 0;
        cells[0].Filled = true;
    }
}

Action solver() {
    if (init_phase){
        init_grid();
        init_phase = false;
    } 
    return floodFill();
}

bool detectWalls()
{
    bool new_wall_detected = false;
    char wall_direction;
    if (API_wallFront())
    {
        if (bot_x_velocity == 1)
        {
            wall_direction = 'e';
            if (!cells[bot_x_pos + LENGTH * bot_y_pos].walls[1]) {
                cells[bot_x_pos + LENGTH * bot_y_pos].walls[1] = true;
                new_wall_detected = true;
            }
        }
        else if (bot_x_velocity == -1)
        {
            wall_direction = 'w';
            if (!cells[bot_x_pos + LENGTH * bot_y_pos].walls[3]) {
                cells[bot_x_pos + LENGTH * bot_y_pos].walls[3] = true;
                new_wall_detected = true;
            }
        }
        else if (bot_y_velocity == 1)
        {
            wall_direction = 'n';
            if (!cells[bot_x_pos + LENGTH * bot_y_pos].walls[0]) {
                cells[bot_x_pos + LENGTH * bot_y_pos].walls[0] = true;
                new_wall_detected = true;
            }
        }
        else if (bot_y_velocity == -1)
        {
            wall_direction = 's';
            if (!cells[bot_x_pos + LENGTH * bot_y_pos].walls[2]) {
                cells[bot_x_pos + LENGTH * bot_y_pos].walls[2] = true;
                new_wall_detected = true;
            }
            
        }
    }

    if (API_wallLeft())
    {
        if (bot_x_velocity == 1)
        {
            wall_direction = 'n';
            if (!cells[bot_x_pos + LENGTH * bot_y_pos].walls[0]) {
                cells[bot_x_pos + LENGTH * bot_y_pos].walls[0] = true;
                new_wall_detected = true;
            }
        }
        else if (bot_x_velocity == -1)
        {
            wall_direction = 's';
            if (!cells[bot_x_pos + LENGTH * bot_y_pos].walls[2]) {
                cells[bot_x_pos + LENGTH * bot_y_pos].walls[2] = true;
                new_wall_detected = true;
            }
        }
        else if (bot_y_velocity == 1)
        {
            wall_direction = 'w';
            if (!cells[bot_x_pos + LENGTH * bot_y_pos].walls[3]) {
                cells[bot_x_pos + LENGTH * bot_y_pos].walls[3] = true;
                new_wall_detected = true;
            }
        }
        else if (bot_y_velocity == -1)
        {
            wall_direction = 'e';
            if (!cells[bot_x_pos + LENGTH * bot_y_pos].walls[1]) {
                cells[bot_x_pos + LENGTH * bot_y_pos].walls[1] = true;
                new_wall_detected = true;
            }
        }
    }

    if (API_wallRight())
    {
        if (bot_x_velocity == 1)
        {
            wall_direction = 's';
            if (!cells[bot_x_pos + LENGTH * bot_y_pos].walls[2]) {
                cells[bot_x_pos + LENGTH * bot_y_pos].walls[2] = true;
                new_wall_detected = true;
            }
        }
        else if (bot_x_velocity == -1)
        {
            wall_direction = 'n';
            if (!cells[bot_x_pos + LENGTH * bot_y_pos].walls[0]) {
                cells[bot_x_pos + LENGTH * bot_y_pos].walls[0] = true;
                new_wall_detected = true;
            }
        }
        else if (bot_y_velocity == 1)
        {
            wall_direction = 'e';
            if (!cells[bot_x_pos + LENGTH * bot_y_pos].walls[1]) {
                cells[bot_x_pos + LENGTH * bot_y_pos].walls[1] = true;
                new_wall_detected = true;
            }
        }
        else if (bot_y_velocity == -1)
        {
            wall_direction = 'w';
            if (!cells[bot_x_pos + LENGTH * bot_y_pos].walls[3]) {
                cells[bot_x_pos + LENGTH * bot_y_pos].walls[3] = true;
                new_wall_detected = true;
            }
        }
    }

    return new_wall_detected;
}

Action decideBestMove() {
    int left_cell_offset[2] = {0};
    int left_cell_distance;
    int right_cell_distance;
    int front_cell_distance;
    if (bot_x_velocity == -1) {
        wall_left = cells[bot_x_pos + bot_y_pos * LENGTH].walls[2];
        wall_right = cells[bot_x_pos + bot_y_pos * LENGTH].walls[0];
        wall_front = cells[bot_x_pos + bot_y_pos * LENGTH].walls[3];
        left_cell_offset[0] = 0;
        left_cell_offset[1] = -1;
    }
    else if (bot_x_velocity == 1) {
        wall_left = cells[bot_x_pos + bot_y_pos * LENGTH].walls[0];
        wall_right = cells[bot_x_pos + bot_y_pos * LENGTH].walls[2];
        wall_front = cells[bot_x_pos + bot_y_pos * LENGTH].walls[1];
        left_cell_offset[0] = 0;
        left_cell_offset[1] = 1;
    }
    else if (bot_y_velocity == -1) {
        wall_left = cells[bot_x_pos + bot_y_pos * LENGTH].walls[1];
        wall_right = cells[bot_x_pos + bot_y_pos * LENGTH].walls[3];
        wall_front = cells[bot_x_pos + bot_y_pos * LENGTH].walls[2];
        left_cell_offset[0] = 1;
        left_cell_offset[1] = 0;
    }
    else {
        wall_left = cells[bot_x_pos + bot_y_pos * LENGTH].walls[3];
        wall_right = cells[bot_x_pos + bot_y_pos * LENGTH].walls[1];
        wall_front = cells[bot_x_pos + bot_y_pos * LENGTH].walls[0];
        left_cell_offset[0] = -1;
        left_cell_offset[1] = 0;
    }

    //sprintf(str, "wall_front: %d, wall_left: %d, wall_right: %d", wall_front, wall_left, wall_right);
    //debug_log(str);

    if (wall_left)
        left_cell_distance = 1000;
    else
        left_cell_distance = cells[(bot_x_pos + left_cell_offset[0]) + LENGTH * (bot_y_pos + left_cell_offset[1])].Distance;
    //sprintf(str, "%d", (bot_x_pos + left_cell_offset[0]) + LENGTH * (bot_y_pos + left_cell_offset[1]));
    //debug_log(str);
    //sprintf(str, "Left Cell Coordinates are (%d, %d) with distance: %d", bot_x_pos + left_cell_offset[0], bot_y_pos + left_cell_offset[1], left_cell_distance);
    //debug_log(str);

    if (wall_right)
        right_cell_distance = 1000;
    else
        right_cell_distance = cells[(bot_x_pos - left_cell_offset[0]) + LENGTH * (bot_y_pos - left_cell_offset[1])].Distance;
    //sprintf(str, "Right Cell Coordinates are (%d, %d) with distance: %d", bot_x_pos - left_cell_offset[0], bot_y_pos - left_cell_offset[1], right_cell_distance);
    //debug_log(str);

    if (wall_front)
        front_cell_distance = 1000;
    else
        front_cell_distance = cells[(bot_x_pos + left_cell_offset[1]) + LENGTH * (bot_y_pos - left_cell_offset[0])].Distance;
    //sprintf(str, "%d", (bot_x_pos + left_cell_offset[1]) + LENGTH * (bot_y_pos - left_cell_offset[0]));
    //debug_log(str);
    //sprintf(str, "Front Cell Coordinates are (%d, %d) with distance: %d", bot_x_pos + left_cell_offset[1], bot_y_pos - left_cell_offset[0], front_cell_distance);
    //debug_log(str);


    if (left_cell_distance <= right_cell_distance) {
        if (left_cell_distance < front_cell_distance)
            return LEFT;
    }
    else if (right_cell_distance < left_cell_distance) {
        if (right_cell_distance < front_cell_distance)
            return RIGHT;
    }
    
    return FORWARD;
}


// Put your implementation of floodfill here!
Action floodFill() {
    Action nextMove;

    //if (API_wasReset()) {
    //    API_ackReset();
    //    bot_x_pos = 0;
    //    bot_y_pos = 0;
    //    bot_y_velocity = 1;
    //    bot_x_velocity = 0;
    //}

    if (detectWalls()) {
        recalculateFloodfill();
    }

    nextMove = decideBestMove();

    //sprintf(str, "Existing goal cells: %d", goal_cells_existing);
    //debug_log(str);

    if (!goal_reached) {
        goal_reached = (cells[bot_x_pos + bot_y_pos * LENGTH].Distance == 0);
    }
    else if (goal_cells_found >= goal_cells_existing) {
        goal_reached = false;
        if (goal_cells_existing == 4)
            goal_cells_existing = 1;
        else 
            goal_cells_existing = 4;

        recalculateFloodfill();
        print_goal_message = true;
        goal_cells_found = 0;
    }

    if (goal_reached) {
        goal_cells_found++;
        if (print_goal_message) {
            print_goal_message = false;
        }
    }

    if (nextMove == RIGHT){ 
        //debug_log("TURNING RIGHT.");
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
        //debug_log("TURNING LEFT.");
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
    else if (!API_wallFront()){ 
        //sprintf(str, "x: %d, y: %d", bot_x_pos, bot_y_pos);
        //debug_log(str);
        //sprintf(str, "x_vel: %d, y_vel: %d", bot_x_velocity, bot_y_velocity);
        //debug_log(str);
        bot_x_pos += bot_x_velocity;
        bot_y_pos += bot_y_velocity;
    } 
    else {
        nextMove = LEFT;
        //debug_log("TURNING LEFT.");
        if (bot_y_velocity == 1) {
            bot_x_velocity = -1;
            bot_y_velocity = 0;
        }
        else if (bot_y_velocity == -1) {
            bot_x_velocity = 1;
            bot_y_velocity = 0;
        }
        else {
            if (bot_x_velocity == -1) {
                bot_y_velocity = -1;
            }
            else if (bot_x_velocity == 1) {
                bot_y_velocity = 1;
            }
            bot_x_velocity = 0;
        }
    }

    return nextMove;
    
}