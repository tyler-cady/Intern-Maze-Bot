#include "API.h"
#include <iostream>
#include <cstdlib> // for abs()
#include <stack>
#include <set>
#include <vector>
#include <map>

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

void moveForward(int steps) {
    for (int i = 0; i < steps; i++) {
        API::moveForward();
    }
}

std::vector<std::string> smoothPath(const std::vector<Cell>& path, const std::vector<std::string>& moves) {
    std::vector<std::string> smooth_moves;
    std::map<Cell, int> cell_index;
    std::stack<Cell> cell_stack;

    // Remove loops from path
    for (int i = 0; i < path.size(); ++i) {
        Cell current = path[i];
        if (cell_index.find(current) != cell_index.end()) {
            int loop_start = cell_index[current];
            while (path[loop_start] != current) {
                loop_start++;
            }
            // Remove the loop
            path.erase(path.begin() + loop_start, path.begin() + i);
            i = loop_start;
        }
        cell_index[current] = i;
    }

    // Optimize moves
    for (int i = 0; i < moves.size(); ++i) {
        if (i < moves.size() - 1 && moves[i] == "F" && moves[i + 1] == "F") {
            int count = 0;
            while (i < moves.size() && moves[i] == "F") {
                count++;
                i++;
            }
            smooth_moves.push_back("F" + std::to_string(count));
            i--;
        } else {
            smooth_moves.push_back(moves[i]);
        }
    }

    return smooth_moves;
}

void h_dfs() {
    std::stack<Cell> path_stack;
    std::set<Cell> visited;
    std::vector<Cell> path;
    std::vector<std::string> moves;
    Cell current = MAZE_START;
    path_stack.push(current);
    visited.insert(current);
    path.push_back(current);

    while (!path_stack.empty()) {
        current = path_stack.top();
        if (isCenter(current)) {
            std::cout << "Reached center!" << std::endl;
            break;
        }

        bool options[3] = {false, false, false};
        getAvailTurnOptions(options);
        int chosen_direction = -1;
        turn_heuristic(options, current, chosen_direction);

        bool move_successful = false;
        Cell next = current;

        if (chosen_direction != -1) {
            switch (chosen_direction) {
                case 0: // Left
                    API::turnLeft();
                    if (!API::wallFront()) {
                        moveForward(1);
                        next = Cell(current.x - 1, current.y, 0);
                        move_successful = true;
                        moves.push_back("L");
                    } else {
                        API::turnRight(); // Undo the turn
                    }
                    break;
                case 1: // Forward
                    if (!API::wallFront()) {
                        moveForward(1);
                        next = Cell(current.x, current.y + 1, 0);
                        move_successful = true;
                        moves.push_back("F");
                    }
                    break;
                case 2: // Right
                    API::turnRight();
                    if (!API::wallFront()) {
                        moveForward(1);
                        next = Cell(current.x + 1, current.y, 0);
                        move_successful = true;
                        moves.push_back("R");
                    } else {
                        API::turnLeft(); // Undo the turn
                    }
                    break;
                default:
                    break;
            }

            if (move_successful && visited.find(next) == visited.end()) {
                path_stack.push(next);
                visited.insert(next);
                path.push_back(next);
            } else {
                move_successful = false;
                if (!moves.empty()) moves.pop_back(); // Remove the move if it didn't work
            }
        }

        // If no move was successful, backtrack
        if (!move_successful) {
            path_stack.pop();
            if (!path_stack.empty()) {
                Cell back = path_stack.top();
                // Determine the direction to turn back to the previous cell
                if (back.x < current.x) {
                    API::turnRight();
                    API::turnRight();
                    moveForward(1);
                    API::turnRight();
                    API::turnRight();
                } else if (back.x > current.x) {
                    API::turnRight();
                    moveForward(1);
                    API::turnLeft();
                } else if (back.y < current.y) {
                    API::turnLeft();
                    API::turnLeft();
                    moveForward(1);
                    API::turnRight();
                    API::turnRight();
                } else if (back.y > current.y) {
                    moveForward(1);
                }
                if (!moves.empty()) moves.pop_back(); // Remove the move when backtracking
            }
        }
    }

    std::vector<std::string> smooth_moves = smoothPath(path, moves);

    // Print the smooth moves taken
    for (const std::string& move : smooth_moves) {
        std::cout << move << " ";
    }
    std::cout << std::endl;
}

int main() {
    h_dfs();
    return 0;
}