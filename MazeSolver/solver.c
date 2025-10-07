#include "solver.h"
#include "API.h"
#include <stdlib.h>

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
bool goalReached = false;
int iterations = 0;
int bot_y_velocity = 1;
int bot_x_velocity = 0;
int bot_x_pos = 0;
int bot_y_pos = 0;
int left_cell_distance;
int right_cell_distance;
int front_cell_distance;
int runNumber = 0;
char str[20];
char wall_direction;

void init_grid() {
    // Coordinates of the 4 goal cells in a 16x16 maze (center 2x2)
    int goals[4][2] = 
    {
        {7, 7},
        {7, 8},
        {8, 7},
        {8, 8}
    };

    // Fill grid[][] with the Manhattan distance to the nearest goal
    for (int i = 0; i < LENGTH; i++) 
    {
        for (int j = 0; j < LENGTH; j++) {
            int minDist = LENGTH * LENGTH; // large initial value
            for (int g = 0; g < 4; g++) {
                int gx = goals[g][0]; // calculating the minimum distances for each cell
                int gy = goals[g][1];
                int dist = abs(i - gx) + abs(j - gy);
                if (dist < minDist) minDist = dist;
            }
            grid[i][j] = minDist;
        }
    }
    
    reset_cells();
    print_grid();
    // debug_log("Maze initially filled.");
}

void print_grid(){
    //debug_log("UPDATING MANHATTAN DISTANCES");
    for (int i = 0; i < LENGTH; i++){
        for (int j = 0; j < LENGTH; j++){
            sprintf(str, "%d", grid[i][j]);
            API_setText(i, j, str);
        }
    }
}

void recalculateFloodfill() {
    // STEP 1: Reset all distances
    // We loop through every cell in the maze grid.
    // Each cell is temporarily set to a very large number (LENGTH * LENGTH).
    // This means "very far away" before we recalculate. -> remember from ee259 we did something like this for finding the minimum
    // We also reset the "cellFilled" array to false, so the BFS can mark them later.
    for (int i = 0; i < LENGTH; i++) {
        for (int j = 0; j < LENGTH; j++) {
            int idx = j + i * LENGTH; // Convert (x=j, y=i) into a 1D array index 
            cells[idx].Distance = LENGTH * LENGTH; // Initialize as very far (like ee259, set a high value to find a new minimum)
            cellFilled[idx] = false;               // Reset fill status
        }
    }

    // STEP 2: Set up  queue
    // using two arrays (qx, qy) to store x and y coordinates.
    // "front" is where we pop from the queue, "back" is where we push new cells in.
    int qx[MAX_QUEUE], qy[MAX_QUEUE];
    int front = 0, back = 0;

    // STEP 3: Define the goal cells
    // These four coordinates are the "end points" the mouse wants to reach.
    // Their distance is set to 0 because they are the target cells.
    int goals[4][2] = {{7,7}, {7,8}, {8,7}, {8,8}};
    for (int g = 0; g < 4; g++) {
        int gx = goals[g][0];
        int gy = goals[g][1];
        int idx = gx + LENGTH * gy;
        cells[idx].Distance = 0;   // Distance = 0 at the goal
        qx[back] = gx;             // Push goal x into queue
        qy[back] = gy;             // Push goal y into queue
        back++;                    // Increase queue size
    }

    // STEP 4: Floodfill
    // spread outward from the goal cells 
    // Each step increases distance by +1, filling the whole maze with shortest path distances.
    while (front < back) {
        // Take the next cell from the queue
        int cx = qx[front];  // current x
        int cy = qy[front];  // current y
        int cidx = cx + cy * LENGTH; // index of current cell
        front++; // move the "front" forward to make room for new cells

        int currDist = cells[cidx].Distance; // Current cell’s distance

        // Explore neighbors in 4 directions
        // dx, dy arrays represent movement North, East, South, West.
        // For example, dir=0 → (0,1) in terms of (dx,dy)= North.

        int dx[4] = {0, 1, 0, -1}; // These 2 lines represent the 4 possible directions
        int dy[4] = {1, 0, -1, 0};

        for (int dir = 0; dir < 4; dir++) { // looping through each direction

            int nx = cx + dx[dir]; // Neighbor x
            int ny = cy + dy[dir]; // Neighbor y

            // Skip neighbors that are outside the maze boundaries
            if (nx < 0 || ny < 0 || nx >= LENGTH || ny >= LENGTH) continue;

            int nidx = nx + ny * LENGTH; // get Neighbor index

            // Check for walls before moving
            // If there’s a wall in the current cell pointing in "dir", we cannot move there
            if (cells[cidx].walls[dir]) continue; // continue if there is a wall in "dir" at cell index (current cell)

            // Also check the neighbor cell: does it have a wall facing back at us?
            // ex: If we try to go East, we also make sure the neighbor’s West wall is open.
            int opposite = (dir + 2) % 4; // Opposite direction (0<->2, 1<->3)
            if (cells[nidx].walls[opposite]) continue;

            //Update neighbor distance if shorter path is found
            if (cells[nidx].Distance > currDist + 1) {
                cells[nidx].Distance = currDist + 1; // Set distance
                qx[back] = nx; // Push neighbor into BFS queue
                qy[back] = ny;
                back++;
            }
        }
    }
    // This is just displaying the flood fill distances on the maze display
    for (int i = 0; i < LENGTH; i++) {
        for (int j = 0; j < LENGTH; j++) {
            sprintf(str, "%d", cells[j + i * LENGTH].Distance); // Convert number to text
            API_setText(j, i, str); // Show the text on the maze cell
        }
    }
}

//This is pretty simple, im pretty sure u can just replace the goal cells with (0,0) and it will work the same way
void recalculateFloodfill2() {
    for (int i = 0; i < LENGTH; i++) {
        for (int j = 0; j < LENGTH; j++) {
            int idx = j + i * LENGTH; 
            cells[idx].Distance = LENGTH * LENGTH; 
            cellFilled[idx] = false;
        }
    }

    int qx[MAX_QUEUE], qy[MAX_QUEUE];
    int front = 0, back = 0;

    int goal[1][2] = {0,0};
    for (int g = 0; g < 1; g++) {
        int gx = goal[g][0];
        int gy = goal[g][1];
        int idx = gx + LENGTH * gy;
        cells[idx].Distance = 0;   
        qx[back] = gx;             
        qy[back] = gy;            
        back++;                    
    }

    while (front < back) {
        int cx = qx[front];  
        int cy = qy[front];  
        int cidx = cx + cy * LENGTH; 
        front++; 

        int currDist = cells[cidx].Distance; 

        int dx[4] = {0, 1, 0, -1};
        int dy[4] = {1, 0, -1, 0};

        for (int dir = 0; dir < 4; dir++) {
            int nx = cx + dx[dir]; 
            int ny = cy + dy[dir]; 
            if (nx < 0 || ny < 0 || nx >= LENGTH || ny >= LENGTH) continue;

            int nidx = nx + ny * LENGTH; 


            if (cells[cidx].walls[dir]) continue;

            int opposite = (dir + 2) % 4; 
            if (cells[nidx].walls[opposite]) continue;

            if (cells[nidx].Distance > currDist + 1) {
                cells[nidx].Distance = currDist + 1; 
                qx[back] = nx; 
                qy[back] = ny;
                back++;
            }
        }
    }

    for (int i = 0; i < LENGTH; i++) {
        for (int j = 0; j < LENGTH; j++) {
            sprintf(str, "%d", cells[j + i * LENGTH].Distance); 
            API_setText(j, i, str); 
        }
    }
}

 //just use api to check each direction and mark walls
void detectWalls() 
{
    if(API_wallFront())
    {
        if(bot_x_velocity == 1)
        {
            wall_direction = 'e';
            cells[bot_x_pos + LENGTH * bot_y_pos].walls[1] = true;
            API_setWall(bot_x_pos, bot_y_pos, wall_direction);
        }
        else if (bot_x_velocity == -1)
        {
            wall_direction = 'w';
            cells[bot_x_pos + LENGTH * bot_y_pos].walls[3] = true;
            API_setWall(bot_x_pos, bot_y_pos, wall_direction);
        }
        else if (bot_y_velocity == 1)
        {
            wall_direction = 'n';
            cells[bot_x_pos + LENGTH * bot_y_pos].walls[0] = true;
            API_setWall(bot_x_pos, bot_y_pos, wall_direction);
        }
        else if (bot_y_velocity == -1)
        {
            wall_direction = 's';
            cells[bot_x_pos + LENGTH * bot_y_pos].walls[2] = true;
            API_setWall(bot_x_pos, bot_y_pos, wall_direction);
        }
    }

    if(API_wallLeft())
    {
        if(bot_x_velocity == 1)
        {
            wall_direction = 'n';
            cells[bot_x_pos + LENGTH * bot_y_pos].walls[0] = true;
            API_setWall(bot_x_pos, bot_y_pos, wall_direction);
        }
        else if (bot_x_velocity == -1)
        {
            wall_direction = 's';
            cells[bot_x_pos + LENGTH * bot_y_pos].walls[2] = true;
            API_setWall(bot_x_pos, bot_y_pos, wall_direction);
        }
        else if (bot_y_velocity == 1)
        {
            wall_direction = 'w';
            cells[bot_x_pos + LENGTH * bot_y_pos].walls[3] = true;
            API_setWall(bot_x_pos, bot_y_pos, wall_direction);
        }
        else if (bot_y_velocity == -1)
        {
            wall_direction = 'e';
            cells[bot_x_pos + LENGTH * bot_y_pos].walls[1] = true;
            API_setWall(bot_x_pos, bot_y_pos, wall_direction);
        }
    }

    if(API_wallRight())
    {
        if(bot_x_velocity == 1)
        {
            wall_direction = 's';
            cells[bot_x_pos + LENGTH * bot_y_pos].walls[2] = true;
            API_setWall(bot_x_pos, bot_y_pos, wall_direction);
        }
        else if (bot_x_velocity == -1)
        {
            wall_direction = 'n';
            cells[bot_x_pos + LENGTH * bot_y_pos].walls[0] = true;
            API_setWall(bot_x_pos, bot_y_pos, wall_direction);
        }
        else if (bot_y_velocity == 1)
        {
            wall_direction = 'e';
            cells[bot_x_pos + LENGTH * bot_y_pos].walls[1] = true;
            API_setWall(bot_x_pos, bot_y_pos, wall_direction);
        }
        else if (bot_y_velocity == -1)
        {
            wall_direction = 'w';
            cells[bot_x_pos + LENGTH * bot_y_pos].walls[3] = true;
            API_setWall(bot_x_pos, bot_y_pos, wall_direction);
        }
    }
}

 //just using api to mark the outer walls (basically looping through the outer edges of the matrix)
void outerWalls() 
{
    for (int i = 0; i < LENGTH; i++){
        for (int j = 0; j < LENGTH; j++){
            if(i == 0) { // bottom
                cells[j+LENGTH*i].walls[2] = true;
                API_setWall(j, i, 's');
            }
            if(i == LENGTH-1) { // top row
                cells[j+LENGTH*i].walls[0] = true;
                API_setWall(j, i, 'n');
            }
            if(j == 0) { // left column
                cells[j+LENGTH*i].walls[3] = true;
                API_setWall(j, i, 'w');
            }
            if(j == LENGTH-1) { // right column
                cells[j+LENGTH*i].walls[1] = true;
                API_setWall(j, i, 'e');
            }
        }
    }
}

//This one just saves the manhattan distances into the cells array and sets the middle goals as 0 (8 + (7 * LENGTH) is (7,8) in 2D coordinates [index 120], and so on for the other 3 goals)
void reset_cells()
{
    outerWalls();
    // sprintf(str, "Resetting Cells");
    // debug_log(str);

    for (int i = 0; i < LENGTH; i++){
        for (int j = 0; j < LENGTH; j++){
            int index = j + i * LENGTH;  // This line is converting 2D coordinates to 1D index ex: if u do 0,0 it will be 0 + 0*16 = 0 if you do 1,1 it will be 1 + 1*16 = 17th index in the array.
            cells[index].Distance = grid[i][j];
            cells[index].Coordinate[0] = j;
            cells[index].Coordinate[1] = i;
            cellFilled[index] = false;
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


//I just left this one as Laith left it.
Action solver() {
    if (init){
        init_grid();
        init = false;
    } 
    return floodFill();
}

// No need to worry about this one, but the code is simple enough to follow
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
Action floodFill() 
{
    int INDEX = bot_x_pos + bot_y_pos * LENGTH;
    // Step 1: Detect and mark walls
    if (goalReached == false && runNumber <3) {
            detectWalls();

    // Step 2: Recalculate floodfill distances after wall updates
    recalculateFloodfill();

    // Step 3: Choose the neighbor with the lowest distance
    int bestDist = cells[INDEX].Distance;
    Action bestMove = IDLE;  

    // Four directions: N,E,S,W [0 1 2 3]
    int dx[4] = {0, 1, 0, -1};
    int dy[4] = {1, 0, -1, 0};
    Action dirAction[4] = {FORWARD, RIGHT, IDLE, LEFT}; 

    for (int dir = 0; dir < 4; dir++) { // Neighbor direction
        int nx = bot_x_pos + dx[dir];
        int ny = bot_y_pos + dy[dir];

        if (nx < 0 || ny < 0 || nx >= LENGTH || ny >= LENGTH) continue; // Skip out-of-bounds

        int nidx = nx + ny * LENGTH; // Neighbor index (exact same logic as 2D to 1D conversion)

        // Check if wall blocks movement in this direction & and opposite direction

        if (cells[INDEX].walls[dir]) continue; // cells[index].walls[dir] is a boolean that is true if there is a wall in that direction.

        int opposite = (dir + 2) % 4; // Opposite direction (0<->2, 1<->3) North<->South, East<->West

        if (cells[nidx].walls[opposite]) continue;

        if (cells[nidx].Distance < bestDist) { // if neighbor distance is better, bestDist = neighbor distance, we will move there
            bestDist = cells[nidx].Distance;

            // Decide movement relative to current velocity (if we found a new best move)
            if (dir == 0) { // NORTH
                if (bot_x_velocity == 0 && bot_y_velocity == 1) bestMove = FORWARD;
                else if (bot_x_velocity == 1 && bot_y_velocity == 0) bestMove = LEFT;
                else if (bot_x_velocity == -1 && bot_y_velocity == 0) bestMove = RIGHT;
                else if (bot_x_velocity == 0 && bot_y_velocity == -1) bestMove = LEFT; // U-turn
            }
            else if (dir == 1) { // EAST
                if (bot_x_velocity == 1 && bot_y_velocity == 0) bestMove = FORWARD;
                else if (bot_y_velocity == 1) bestMove = RIGHT;
                else if (bot_y_velocity == -1) bestMove = LEFT;
                else if (bot_x_velocity == -1) bestMove = LEFT; // U-turn
            }
            else if (dir == 2) { // SOUTH
                if (bot_y_velocity == -1) bestMove = FORWARD;
                else if (bot_x_velocity == 1) bestMove = RIGHT;
                else if (bot_x_velocity == -1) bestMove = LEFT;
                else if (bot_y_velocity == 1) bestMove = LEFT; // U-turn
            }
            else if (dir == 3) { // WEST
                if (bot_x_velocity == -1) bestMove = FORWARD;
                else if (bot_y_velocity == 1) bestMove = LEFT;
                else if (bot_y_velocity == -1) bestMove = RIGHT;
                else if (bot_x_velocity == 1) bestMove = LEFT; // U-turn
            }
        }
    }

    // Apply movement and update bot position/heading since velocity is either (1,0), (0,1), (-1,0), (0,-1), we can update like this LAITH IS A
    if (bestMove == FORWARD) {
        bot_x_pos += bot_x_velocity;
        bot_y_pos += bot_y_velocity;
    } 
    else if (bestMove == LEFT) {
        // Rotate left
        int temp = bot_x_velocity;
        bot_x_velocity = -bot_y_velocity;
        bot_y_velocity = temp;
    } 
    else if (bestMove == RIGHT) {
        // Rotate right
        int temp = bot_x_velocity;
        bot_x_velocity = bot_y_velocity;
        bot_y_velocity = -temp;
    } 
    else if (bestMove == IDLE) {
        debug_log("Goal Reached!");
        goalReached = true;
        runNumber++;
    } 
    return bestMove;
    }

    // Logic is the exact same, just using the 2nd recalculate floodfill function.
    else if (goalReached == true) {
            detectWalls();

    recalculateFloodfill2();

    int bestDist = cells[INDEX].Distance;
    Action bestMove = IDLE;  

    int dx[4] = {0, 1, 0, -1};
    int dy[4] = {1, 0, -1, 0};
    Action dirAction[4] = {FORWARD, RIGHT, IDLE, LEFT}; 

    for (int dir = 0; dir < 4; dir++) { 
        int nx = bot_x_pos + dx[dir];
        int ny = bot_y_pos + dy[dir];
        if (nx < 0 || ny < 0 || nx >= LENGTH || ny >= LENGTH) continue; 

        int nidx = nx + ny * LENGTH; 
        if (cells[INDEX].walls[dir]) continue; 
        int opposite = (dir + 2) % 4; 
        if (cells[nidx].walls[opposite]) continue;

        if (cells[nidx].Distance < bestDist) { 
            bestDist = cells[nidx].Distance;

            if (dir == 0) { 
                if (bot_x_velocity == 0 && bot_y_velocity == 1) bestMove = FORWARD;
                else if (bot_x_velocity == 1 && bot_y_velocity == 0) bestMove = LEFT;
                else if (bot_x_velocity == -1 && bot_y_velocity == 0) bestMove = RIGHT;
                else if (bot_x_velocity == 0 && bot_y_velocity == -1) bestMove = LEFT; 
            }
            else if (dir == 1) { 
                if (bot_x_velocity == 1 && bot_y_velocity == 0) bestMove = FORWARD;
                else if (bot_y_velocity == 1) bestMove = RIGHT;
                else if (bot_y_velocity == -1) bestMove = LEFT;
                else if (bot_x_velocity == -1) bestMove = LEFT; 
            }
            else if (dir == 2) { 
                if (bot_y_velocity == -1) bestMove = FORWARD;
                else if (bot_x_velocity == 1) bestMove = RIGHT;
                else if (bot_x_velocity == -1) bestMove = LEFT;
                else if (bot_y_velocity == 1) bestMove = LEFT; 
            }
            else if (dir == 3) { 
                if (bot_x_velocity == -1) bestMove = FORWARD;
                else if (bot_y_velocity == 1) bestMove = LEFT;
                else if (bot_y_velocity == -1) bestMove = RIGHT;
                else if (bot_x_velocity == 1) bestMove = LEFT;
            }
        }
    }

    if (bestMove == FORWARD) {
        bot_x_pos += bot_x_velocity;
        bot_y_pos += bot_y_velocity;
    } 
    else if (bestMove == LEFT) {
        int temp = bot_x_velocity;
        bot_x_velocity = -bot_y_velocity;
        bot_y_velocity = temp;
    } 
    else if (bestMove == RIGHT) {
        int temp = bot_x_velocity;
        bot_x_velocity = bot_y_velocity;
        bot_y_velocity = -temp;
    } 
    else if (bestMove == IDLE) {
        debug_log("Beginning Reached!");
        goalReached = false;
        runNumber++;
    }
    return bestMove;

    }

}