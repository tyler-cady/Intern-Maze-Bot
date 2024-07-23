#include <iostream>
#include <vector>
#include "API.h"
#include "Flood.h"

#define MAZE_SIZE 9



int main() {
    API api;

    int width = api.mazeWidth();
    int height = api.mazeHeight();

    int maze[MAZE_SIZE][MAZE_SIZE] = {0};

    Cell start = {0, 0, 0}; // Example start position
    Cell end = {width - 1, height - 1, 0};

    floodFill(maze, start, end);

    while (!api.wasReset()) {
        if (api.wallFront()) {
            api.turnLeft();
            api.moveForward(1);
        } else {
            api.moveForward(1);
        }
    }

    api.ackReset();
    return 0;
}