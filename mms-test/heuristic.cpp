#include "API.h"
#include <iostream>
#include <cstdlib> // for abs()
#include <stack>
#include <set>

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
        if (cell == center) return true;
    }
    return false;
}

void h_dfs() {
    std::stack<Cell> path;
    std::set<Cell> visited;
    Cell current = MAZE_START;
    path.push(current);
    visited.insert(current);

    while (!path.empty()) {
        current = path.top();
        if (isCenter(current)) {
            std::cout << "Reached center!" << std::endl;
            break;
        }

        bool options[3] = {false, false, false};
        getAvailTurnOptions(options);
        int chosen_direction = -1;
        turn_heuristic(options, current, chosen_direction);

        bool move_successful = false;
        while (!move_successful && chosen_direction != -1) {
            Cell next = current;
            switch (chosen_direction) {
                case 0: // Left
                    API::turnLeft();
                    if (!API::wallFront()) {
                        API::moveForward();
                        next.x -= 1;
                        move_successful = true;
                    } else {
                        API::turnRight(); // Undo the turn
                        options[0] = false; // Mark left as not available
                    }
                    break;
                case 1: // Forward
                    if (!API::wallFront()) {
                        API::moveForward();
                        next.y += 1;
                        move_successful = true;
                    } else {
                        options[1] = false; // Mark forward as not available
                    }
                    break;
                case 2: // Right
                    API::turnRight();
                    if (!API::wallFront()) {
                        API::moveForward();
                        next.x += 1;
                        move_successful = true;
                    } else {
                        API::turnLeft(); // Undo the turn
                        options[2] = false; // Mark right as not available
                    }
                    break;
                default:
                    break;
            }

            // If the move was successful and the cell is not visited, add the next cell to the path and visited set
            if (move_successful && visited.find(next) == visited.end()) {
                path.push(next);
                visited.insert(next);
            } else { // Recalculate turn heuristic if the chosen direction didn't work out or the cell is visited
                move_successful = false;
                turn_heuristic(options, current, chosen_direction);
            }
        }

        // If no move was successful, backtrack
        if (!move_successful) {
            path.pop();
            if (!path.empty()) {
                Cell back = path.top();
                // Determine the direction to turn back to the previous cell
                if (back.x < current.x) {
                    API::turnRight();
                    API::turnRight();
                    API::moveForward();
                    API::turnLeft();
                    API::turnLeft();
                } else if (back.x > current.x) {
                    API::turnRight();
                    API::moveForward();
                    API::turnRight();
                    API::turnRight();
                    API::turnLeft();
                } else if (back.y < current.y) {
                    API::turnLeft();
                    API::moveForward();
                    API::turnLeft();
                } else if (back.y > current.y) {
                    API::turnLeft();
                    API::moveForward();
                    API::turnRight();
                }
            }
        }
    }
}

int main() {
    h_dfs();
    return 0;
}