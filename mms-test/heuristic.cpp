#include "API.h"
#include <iostream>
#include <cstdlib> // for abs()

const int MAZE_SIZE = 9;

struct Cell {
    int x, y;
    int distance;
    Cell(int x, int y, int distance) : x(x), y(y), distance(distance) {}
};

const Cell MAZE_START(0, 0, 0);
const Cell MAZE_CENTERS[4] = {
    Cell(4, 4, 0),
    Cell(4, 5, 0),
    Cell(5, 4, 0),
    Cell(5, 5, 0)
};

void turn_heuristic(bool options[3], Cell current) {
    // Find the closest center to the current cell
    Cell nearest_center = MAZE_CENTERS[0];
    if (current.x <= 4) {
        nearest_center = (current.y <= 4) ? MAZE_CENTERS[0] : MAZE_CENTERS[1];
    } else {
        nearest_center = (current.y <= 4) ? MAZE_CENTERS[2] : MAZE_CENTERS[3];
    }

    // Find the direction to the nearest center from options 
    // Options: left, forward, right 
    Cell avail_cells[3] = { Cell(-1, -1, -1), Cell(-1, -1, -1), Cell(-1, -1, -1) };

    if (options[0]) { // left
        avail_cells[0] = Cell(current.x - 1, current.y, 0);
    }
    if (options[1]) { // forward 
        avail_cells[1] = Cell(current.x, current.y + 1, 0);
    }
    if (options[2]) { // right 
        avail_cells[2] = Cell(current.x + 1, current.y, 0);
    }

    // Find the direction to the nearest center from avail_cells
    int min_distance = 100;
    int min_index = -1;

    for (int i = 0; i < 3; i++) {
        if (avail_cells[i].x != -1 && avail_cells[i].y != -1) {
            int distance = std::abs(avail_cells[i].x - nearest_center.x) + std::abs(avail_cells[i].y - nearest_center.y);
            if (distance < min_distance) {
                min_distance = distance;
                min_index = i;
            }
        }
    }

    // Print the direction to the nearest center
    switch (min_index) {
        case 0:
            std::cout << "L" << std::endl;
            break;
        case 1:
            std::cout << "F" << std::endl;
            break;
        case 2:
            std::cout << "R" << std::endl;
            break;
        default:
            std::cout << "No valid direction" << std::endl;
            break;
    }
}
bool getAvailTurnOptions(){
    bool options[3] = { false, false, false };
        if (!API::wallFront()) {
            options[1] = true;
        }
        if (!API::wallRight()) {
            options[2] = true;
        }
        if (!API::wallLeft()) {
            options[0] = true;
        }
    return options; 
}
void h_dfs() {
    // check wallFront, wallRight, wallLeft and populate options
    bool solved = false;
    while(solved == false){
        bool options[3] = getAvailTurnOptions();
        Cell current(API::mazeWidth(), API::mazeHeight(), 0);
        turn_heuristic(options, current);
        if (API::wallFront()) {
            API::turnRight();
            API::moveForward();
            options[3] = getAvailTurnOptions();
        } else {
            API::moveForward();
            options[3] = getAvailTurnOptions();
        }
    
    }
}

int main() {
    h_dfs();
    return 0;
}
