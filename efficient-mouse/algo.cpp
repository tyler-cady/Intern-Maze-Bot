#include "API.h"
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <array>

#define loopCost 20
#define travelCost 5
#define branchReward 5

#define MAZE_SIZE 9 // Update to 9 for a 9x9 maze

using std::cerr;
using std::string;

int score[MAZE_SIZE][MAZE_SIZE];      // scores every square, initialised to 0
int history[MAZE_SIZE][MAZE_SIZE][3]; // stores last 3 steps of each point

int x = 0, y = 0, facing = 0;  // coordinates & direction faced (initialize to starting point and facing direction)
int moveCount = 1; // total number of moves made till now
bool inDeadEnd = false;    // checks if we are inside a deadend

// Vector to store turns
std::vector<std::array<bool, 4>> turns; // [L, R, F, B]
std::vector<std::array<bool, 4>> optimizePath(const std::vector<std::array<bool, 4>>& originalPath);

// Function declarations
void setWall(int dir, char relativeDir); // sets wall
void think();                            // movement logic (greed)
void move(char relativeDir);              // to move f, l, r

/* this is the main method */
int main(int argc, char *argv[]) {
    std::cerr << "Running..." << std::endl;
    API::setColor(0, 0, 'G');
    API::setText(0, 0, "abc");

    // Initialize score array and display it
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            score[i][j] = 128 - (abs(2 * i - (MAZE_SIZE - 1)) + abs(2 * j - (MAZE_SIZE - 1))) / 2;
            API::setText(i, j, std::to_string(score[i][j]));
        }
    }

    // Main loop to navigate through the maze until reaching the central area
    while (!(x >= 4 && x <= 5 && y >= 3 && y <= 4)) {
        think();
        // Log the current state
        std::cerr << "Position: (" << x << ", " << y << "), Facing: " << facing << std::endl;
        std::cerr << "Score: " << score[x][y] << std::endl;
    }

    // Print the recorded turns as a string
    std::cerr << "Maze solved! Turns taken:" << std::endl;
    std::string seeked;
    for (const auto& turn : turns) {
        if      (turn[0]) seeked += "L";
        else if (turn[1]) seeked += "R";
        else if (turn[2]) seeked += "F";
        else if (turn[2]) seeked += "B";
    }
    std::cerr << seeked << std::endl; 


    // Optimize the path
    auto optimizedTurns = optimizePath(turns);

    std::string opt;
    // Print the optimized path
    std::cerr << "Optimized path:" << std::endl;
    for (const auto& turn : turns) {
        if      (turn[0]) opt += "L";
        else if (turn[1]) opt += "R";
        else if (turn[2]) opt += "F";
        else if (turn[2]) opt += "B";
    }
    std::cerr << opt << std::endl; 
    return 0;
}


/* sets the wall for the cell and adjacent cell */
void setWall(int dir, char relativeDir) {
    if (relativeDir == 'f')
        ; // do nothing
    else if (relativeDir == 'l')
        dir--; // rotate anticlockwise
    else if (relativeDir == 'r')
        dir++; // rotate clockwise

    dir = (dir + 4) % 4; // -1 --> 3 and 4 --> 0

    char absoluteDir = (dir == 0) ? 'n' : (dir == 1) ? 'e'
                                      : (dir == 2)   ? 's'
                                                     : 'w';
    API::setWall(x, y, absoluteDir);
}

/* the main logic used for making decisions at squares */
void think() {
    bool wallFront = API::wallFront(),
         wallLeft = API::wallLeft(),
         wallRight = API::wallRight();
    if (wallFront) setWall(facing, 'f');
    if (wallLeft) setWall(facing, 'l');
    if (wallRight) setWall(facing, 'r');

    int wallCount = wallFront + wallRight + wallLeft;

    history[x][y][0] = history[x][y][1]; // discard first value
    history[x][y][1] = history[x][y][2]; // and add current move
    history[x][y][2] = moveCount;        // to end of history

    // if the last 3 times we reached a square were at regular intervals, we are looping
    if (history[x][y][1] - history[x][y][0] == history[x][y][2] - history[x][y][1]) {
        std::cerr << "looping" << std::endl;
        score[x][y] -= loopCost;
        API::setText(x, y, std::to_string(score[x][y]));
    }

    if (wallCount == 3) { // dead end
        std::cerr << "Help :(" << std::endl;
        inDeadEnd = true; // initiate protocol
        move('b');
    } else if (wallCount == 2) { // only one way to go
        if (!wallFront) {
            move('f');
        } else if (!wallLeft) {
            move('l');
        } else {
            move('r');
        }
    } else if (wallCount < 2) {
        inDeadEnd = false; // no longer in dead end if we are thinking

        int dir, u, v;
        int scoreF = -16383, scoreR = -16383, scoreL = -16383;
        if (!wallFront) {
            dir = facing; // dir is straight ahead
            u = (dir == 1) ? x + 1 : (dir == 3) ? x - 1
                                                : x; // take a step
            v = (dir == 0) ? y + 1 : (dir == 2) ? y - 1
                                                : y; // forwards
            scoreF = (u >= 0 && u < MAZE_SIZE && v >= 0 && v < MAZE_SIZE) ? score[u][v] : -16383;
        }
        if (!wallRight) {
            dir = (facing + 1) % 4; // dir is right
            u = (dir == 1) ? x + 1 : (dir == 3) ? x - 1
                                                : x; // take a step
            v = (dir == 0) ? y + 1 : (dir == 2) ? y - 1
                                                : y; // rightwards
            scoreR = (u >= 0 && u < MAZE_SIZE && v >= 0 && v < MAZE_SIZE) ? score[u][v] : -16383;
        }
        if (!wallLeft) {
            dir = (facing + 3) % 4; // dir is left
            u = (dir == 1) ? x + 1 : (dir == 3) ? x - 1
                                                : x; // take a step
            v = (dir == 0) ? y + 1 : (dir == 2) ? y - 1
                                                : y; // leftwards
            scoreL = (u >= 0 && u < MAZE_SIZE && v >= 0 && v < MAZE_SIZE) ? score[u][v] : -16383;
        }

        if (history[x][y][0] == 0 && history[x][y][1] == 0) {
            score[x][y] += branchReward * (2 - wallCount);
            API::setText(x, y, std::to_string(score[x][y]));
        }

        if (scoreF > -8191 && scoreF >= scoreR && scoreF >= scoreL) {
            std::cerr << "go ahead" << std::endl;
            move('f');
        } else if (scoreR > -8191 && scoreR >= scoreL) {
            std::cerr << "take a right" << std::endl;
            move('r');
        } else if (scoreL > -8191) {
            std::cerr << "take a left" << std::endl;
            move('l');
        } else if (scoreF > -8191) { // just move somewhere if good moves not allowed
            std::cerr << "go ahead" << std::endl;
            move('f');
        } else if (scoreL > -8191) {
            std::cerr << "take a left" << std::endl;
            move('l');
        } else if (scoreR > -8191) {
            std::cerr << "take a right" << std::endl;
            move('r');
        } else {
            inDeadEnd = true; // no other ways left to go
            move('b');        // only way left to go
        }
    }
}
std::string optimizePath(std::string seeked) {
    // LBR = B
    // LBS = R
    // RBL = B
    // SBL = R
    // SBS = B
    // LBL = S
    string opt; 
    do {
        for (int i = 0; i < seeked.length(); (i + 3)){
            string temp = seeked[i] + seeked[i+1] + seeked[i+2];
            if      (temp == "LBR") opt += "B";
            else if (temp == "LBS") opt += "R";
            else if (temp == "RBL") opt += "B";
            else if (temp == "SBL") opt += "R";
            else if (temp == "SBS") opt += "B";
            else if (temp == "LBL") opt += "S";
            else opt += temp;
        }
    } while (opt.find("B"));
    return opt;
}

void move(char relativeDir) {
    score[x][y] -= travelCost;
    API::setText(x, y, std::to_string(score[x][y]));
    moveCount++;

    // Record the turn
    std::array<bool, 4> turn = {false, false, false, false}; // [L, R, F, B]
    if (relativeDir == 'b') {
        turn[3] = true; // Back
    } else if (relativeDir == 'r') {
        turn[1] = true; // Right
    } else if (relativeDir == 'l') {
        turn[0] = true; // Left
    } else if (relativeDir == 'f') {
        turn[2] = true; // Front
    }
    turns.push_back(turn);

    if (inDeadEnd) {
        score[x][y] = -16383; // mark as dead end
        API::setText(x, y, "X");
    }

    if (relativeDir == 'b') { // turn around
        API::turnRight();
        API::turnRight();
        facing = (facing + 2) % 4;
    } else if (relativeDir == 'r') { // turn right
        API::turnRight();
        facing = (facing + 1) % 4;
    } else if (relativeDir == 'l') { // turn left
        API::turnLeft();
        facing = (facing + 3) % 4;
    }

    API::moveForward(); // go ahead

    // Update coordinates based on facing direction
    if (facing == 0) y++;    // moving north
    else if (facing == 1) x++; // moving east
    else if (facing == 2) y--; // moving south
    else if (facing == 3) x--; // moving west

    // Ensure coordinates are within maze bounds
    x = std::max(0, std::min(MAZE_SIZE - 1, x));
    y = std::max(0, std::min(MAZE_SIZE - 1, y));
}
