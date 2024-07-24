#include "API.h"
#include <iostream>
#include <cstdlib> // for abs()
#include <stack>
#include <set>
using namespace std;

const int MAZE_SIZE = 9;

struct Cell {
    int x, y;
    int distance;
    Cell(int x, int y, int distance) : x(x), y(y), distance(distance) {}
    bool operator==(const Cell& other) const {
        return x == other.x && y == other.y;
    }
    bool operator<(const Cell& other) const {
        return x < other.x || (x == other.x && y < other.y);
    }
};

const Cell MAZE_START(0, 0, 0);
const Cell MAZE_CENTERS[4] = {
    Cell(4, 4, 0),
    Cell(4, 5, 0),
    Cell(5, 4, 0),
    Cell(5, 5, 0)
};

void turn_heuristic(bool options[3], Cell current, int& chosen_direction) {
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

    if (options[0]) avail_cells[0] = Cell(current.x - 1, current.y, 0);
    if (options[1]) avail_cells[1] = Cell(current.x, current.y + 1, 0);
    if (options[2]) avail_cells[2] = Cell(current.x + 1, current.y, 0);

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

    chosen_direction = min_index;
}

void getAvailTurnOptions(bool options[3]) {
    options[0] = !API::wallLeft();
    options[1] = !API::wallFront();
    options[2] = !API::wallRight();
}

bool isCenter(const Cell& cell) {
    for (const Cell& center : MAZE_CENTERS) {
        if (cell == center) {
            API::setText(center.x, center.y, "SOLVED");
            return true;
        }
    }
    return false;
}
void move(int direction){
    if (direction == 0) {
        API::turnLeft();
    } else if (direction == 2) {
        API::turnRight();
    }
    API::moveForward();
}
int random_choice(bool options[], int& chosen_direction){
    int random = rand() % 3;
    while (!options[random]) {
        random = rand() % 3;
    }
    chosen_direction = random;
    return chosen_direction;
}
void dfs(){ 
    // use depth-first search to find the path (return the path as an array of 3 bool chunks)
    // drive the maze and return the path to the center as an array of bools (left, forward, right)
    
    stack<Cell> path;
    set<Cell> visited;
    Cell current = MAZE_START;
    path.push(current);
    visited.insert(current);
    while (!path.empty() && !isCenter(current)) {
        current = path.top();
        
        bool options[3] = {false, false, false};
        getAvailTurnOptions(options);
        cout << "Options: " << options[0] << " " << options[1] << " " << options[2] << endl;
        if (!options[0] && !options[1] && !options[2]) {
            // Dead end, backtrack
            API::turnLeft();
            API::turnLeft();
            API::moveForward();
            //remove that cell from the path
            path.pop();
            continue;
        }
        int chosen_direction = -1;
        //turn_heuristic(options, current, chosen_direction);
        chosen_direction = random_choice(options, chosen_direction);
        bool move_successful = false;
        move(chosen_direction);
        switch (chosen_direction) {
            case 0: current.x--; break;
            case 1: current.y++; break;
            case 2: current.x++; break;
        }
        path.push(current);
        visited.insert(current);
    }
    
}

int main() {
    //use dfs() to find the path
    std::cout<<"Start"<<std::endl;
    dfs();
    return 0;
}
