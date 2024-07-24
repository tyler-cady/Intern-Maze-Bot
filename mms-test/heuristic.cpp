#include "API.h"
#include <iostream>



const int MAZE_SIZE = 9;
const int STACK_CAPACITY = MAZE_SIZE * MAZE_SIZE;
const int PATH_CAPACITY = 100;

struct Cell {
    int x, y;
    int distance;
};
// cell constructor
Cell(int x, int y, int distance){
    this->x = x;
    this->y = y;
    this->distance = distance;
}

const Cell MAZE_START = {0, 0, 0};
const Cell MAZE_CENTERS[4] ={{4, 4, 0}, {4, 5, 0}, {5, 4, 0}, {5, 5, 0}};

void turn_heuristic(bool options[3], Cell current){
    // find the closest center to current cell
    Cell nearest_center;
    if (current.x <= 4)
        if (current.y <=4 ) nearest_center = MAZE_CENTERS[0];
        else nearest_center = MAZE_CENTERS[1];
    else
        if (current.y <=4 ) nearest_center = MAZE_CENTERS[2];
        else nearest_center = MAZE_CENTERS[3];

    // find the direction to the nearest center from options 
    // options: left, forward, right 
    Cell avail_cells[3];
    if (options[0] == true) { // left
        avail_cells[0] = (current.x - 1, current.y, 0);
    }
    if (options[1] == true) { // forward 
        avail_cells[1] = (current.x, current.y + 1, 0);
    }
    if (options[2] == true) { // right 
        avail_cells[2] = (current.x + 1, current.y, 0);
    }

    // find the direction to the nearest center from avail_cells
    int min_distance = 100;
    int min_index = 0;
    for (int i = 0; i < 3; i++){
        int distance = abs(avail_cells[i].x - nearest_center.x) + abs(avail_cells[i].y - nearest_center.y);
        if (distance < min_distance){
            min_distance = distance;
            min_index = i;
        }
    }
    // print the direction to the nearest center
    switch(min_index){
        case 0:
            std::cout << "L" << std::endl;
            break;
        case 1:
            std::cout << "F" << std::endl;
            break;
        case 2:
            std::cout << "R" << std::endl;
            break;
    }

}
void h_dfs(){
    printf("h_dfs");
}

int main(){
    // API api;
    // int maze[MAZE_SIZE][MAZE_SIZE] = {0};

    // bool solveMode = true;
    // switch(solveMode){
    //     case true:
    //         //heuristic dfs call
    //         break;
    //     case false:
    //         //runMaze(solved_path);
    //         break;
    // }
    // return 0; 

    bool options[3] = {true, true, true};
    Cell current = {4, 0, 0};
    turn_heuristic(options, current);
    return 0;
}
