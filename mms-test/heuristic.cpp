#include "API.h"
#include <iostream>
#include <cstdlib> // for abs()
#include <stack>
#include <set>
#include <vector>
#include <map>
#include <algorithm>

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

void move(int direction) {
    if (direction == 0) {
        API::turnLeft();
    } else if (direction == 2) {
        API::turnRight();
    }
    API::moveForward();
}

int random_choice(bool options[], int& chosen_direction) {
    int random = rand() % 3;
    while (!options[random]) {
        random = rand() % 3;
    }
    chosen_direction = random;
    return chosen_direction;
}

void smoothPath(vector<Cell>& path, vector<string>& moves) {
    // Remove loops
    map<Cell, int> cell_index;
    vector<Cell> smoothed_path;
    vector<string> smoothed_moves;

    for (int i = 0; i < path.size(); ++i) {
        if (cell_index.find(path[i]) != cell_index.end()) {
            int loop_start = cell_index[path[i]];
            path.erase(path.begin() + loop_start, path.begin() + i);
            moves.erase(moves.begin() + loop_start, moves.begin() + i);
            i = loop_start;
        }
        cell_index[path[i]] = i;
    }

    // Combine consecutive moves
    for (int i = 0; i < moves.size(); ++i) {
        if (i < moves.size() - 1 && moves[i] == "F" && moves[i + 1] == "F") {
            int count = 0;
            while (i < moves.size() && moves[i] == "F") {
                count++;
                i++;
            }
            smoothed_moves.push_back("F" + to_string(count));
            i--;
        } else {
            smoothed_moves.push_back(moves[i]);
        }
    }

    // Update path and moves
    path = smoothed_path;
    moves = smoothed_moves;
}

void dfs() {
    stack<Cell> path;
    set<Cell> visited;
    vector<Cell> path_list;
    vector<string> moves;
    Cell current = MAZE_START;
    path.push(current);
    visited.insert(current);
    path_list.push_back(current);

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
            path.pop();
            path_list.pop_back();
            if (!moves.empty()) moves.pop_back();
            continue;
        }

        int chosen_direction = -1;
        chosen_direction = random_choice(options, chosen_direction);
        move(chosen_direction);

        switch (chosen_direction) {
            case 0: current.x--; moves.push_back("L"); break;
            case 1: current.y++; moves.push_back("F"); break;
            case 2: current.x++; moves.push_back("R"); break;
        }

        path.push(current);
        visited.insert(current);
        path_list.push_back(current);
    }

    smoothPath(path_list, moves);

    // Print the smooth moves taken
    for (const string& move : moves) {
        cout << move << " ";
    }
    cout << endl;
}

int main() {
    cout << "Start" << endl;
    dfs();
    return 0;
}