#include "Mouse.h"
#include <Wire.h>

Mouse::Mouse() {
    Wire.begin();
    mpu.initialize();
    
    position[0] = 0;
    position[1] = 0;
    init_maze();
}

void Mouse::turn(float degree) {
    // Implement turning logic using motors and MPU6050
}

void Mouse::fast_turn(float degree) {
    // Implement fast turning logic using motors and MPU6050
}

void Mouse::dash(float distance) {
    // Implement dashing logic using motors and encoders
}

void Mouse::diagonal_dash() {
    // Implement diagonal dashing logic
    float speed = 150; // Example speed
    // Set motor speeds for diagonal dash
    // Example: setMotorSpeeds(leftSpeed, rightSpeed);
}

void Mouse::solve_maze() {
    flood_fill();
    // Pathfinding to reach the goal
    while (!is_goal(position[0], position[1])) {
        int min_value = 1000;
        int next_x = position[0];
        int next_y = position[1];
        
        // Check all four directions and choose the cell with the smallest flood value
        if (position[1] > 0 && flood[position[1] - 1][position[0]] < min_value) { // North
            min_value = flood[position[1] - 1][position[0]];
            next_x = position[0];
            next_y = position[1] - 1;
        }
        if (position[1] < 8 && flood[position[1] + 1][position[0]] < min_value) { // South
            min_value = flood[position[1] + 1][position[0]];
            next_x = position[0];
            next_y = position[1] + 1;
        }
        if (position[0] > 0 && flood[position[1]][position[0] - 1] < min_value) { // West
            min_value = flood[position[1]][position[0] - 1];
            next_x = position[0] - 1;
            next_y = position[1];
        }
        if (position[0] < 8 && flood[position[1]][position[0] + 1] < min_value) { // East
            min_value = flood[position[1]][position[0] + 1];
            next_x = position[0] + 1;
            next_y = position[1];
        }
        
        move_to_cell(next_x, next_y);
    }
    
    // Drive back to start using a different route
    position[0] = 0;
    position[1] = 0;
    flood_fill();
    while (position[0] != 0 || position[1] != 0) {
        int min_value = 1000;
        int next_x = position[0];
        int next_y = position[1];
        
        // Check all four directions and choose the cell with the smallest flood value
        if (position[1] > 0 && flood[position[1] - 1][position[0]] < min_value) { // North
            min_value = flood[position[1] - 1][position[0]];
            next_x = position[0];
            next_y = position[1] - 1;
        }
        if (position[1] < 8 && flood[position[1] + 1][position[0]] < min_value) { // South
            min_value = flood[position[1] + 1][position[0]];
            next_x = position[0];
            next_y = position[1] + 1;
        }
        if (position[0] > 0 && flood[position[1]][position[0] - 1] < min_value) { // West
            min_value = flood[position[1]][position[0] - 1];
            next_x = position[0] - 1;
            next_y = position[1];
        }
        if (position[0] < 8 && flood[position[1]][position[0] + 1] < min_value) { // East
            min_value = flood[position[1]][position[0] + 1];
            next_x = position[0] + 1;
            next_y = position[1];
        }
        
        move_to_cell(next_x, next_y);
    }
}

void Mouse::speedrun() {
    while (true) {
        // Check if a diagonal move is possible
        if (is_diagonalable()) {
            diagonal_dash();
        } else {
            // Use regular dash or turns
            dash(10); // Example distance
        }

        // Update position after movement
        update_position();
        
        // Add logic to break the loop when maze is solved or goal is reached
    }
}

int* Mouse::get_location() {
    return position;
}

int* Mouse::get_valid_turns(int px, int nx, int py, int ny) {
    static int valid_turns[4] = {0}; // {front, left, right, back}
    // Implement logic to determine valid turns based on sensor readings and current position
    return valid_turns;
}

float Mouse::get_distance(int sensor) {
    float distance = 0.0;
    switch (sensor) {
        case 0: // Front sensor
            // Read from FS
            break;
        case 1: // Right sensor
            // Read from RS
            break;
        case 2: // Left sensor
            // Read from LS
            break;
        case 3: // Right Diagonal sensor
            // Read from RDS
            break;
        case 4: // Left Diagonal sensor
            // Read from LDS
            break;
        default:
            // Handle invalid sensor input
            break;
    }
    return distance;
}

void Mouse::flood_fill() {
    // Initialize flood values
    for (int y = 0; y < 9; ++y) {
        for (int x = 0; x < 9; ++x) {
            if (is_goal(x, y)) {
                flood[y][x] = 0;
            } else {
                flood[y][x] = 1000;
            }
        }
    }
    
    // Use a queue to perform the flood fill
    queue<pair<int, int>> q;
    for (int y = 0; y < 9; ++y) {
        for (int x = 0; x < 9; ++x) {
            if (is_goal(x, y)) {
                q.push({x, y});
            }
        }
    }

    while (!q.empty()) {
        int x = q.front().first;
        int y = q.front().second;
        q.pop();

        // Update neighbors
        if (y > 0 && flood[y - 1][x] > flood[y][x] + 1) { // North
            flood[y - 1][x] = flood[y][x] + 1;
            q.push({x, y - 1});
        }
        if (y < 8 && flood[y + 1][x] > flood[y][x] + 1) { // South
            flood[y + 1][x] = flood[y][x] + 1;
            q.push({x, y + 1});
        }
        if (x > 0 && flood[y][x - 1] > flood[y][x] + 1) { // West
            flood[y][x - 1] = flood[y][x] + 1;
            q.push({x - 1, y});
        }
        if (x < 8 && flood[y][x + 1] > flood[y][x] + 1) { // East
            flood[y][x + 1] = flood[y][x] + 1;
            q.push({x + 1, y});
        }
    }
}

void Mouse::init_maze() {
    for (int y = 0; y < 9; ++y) {
        for (int x = 0; x < 9; ++x) {
            maze[y][x] = 0; // Initialize the maze as empty
        }
    }
    // Add walls to the maze representation if needed
}

bool Mouse::is_goal(int x, int y) {
    return (x == 4 && y == 3) || (x == 4 && y == 4) || (x == 5 && y == 3) || (x == 5 && y == 4);
}

void Mouse::move_to_cell(int x, int y) {
    // Move the mouse to the specified cell (x, y) with appropriate turning and dashing
    // Update the current position
    position[0] = x;
    position[1] = y;
}

void Mouse::update_position() {
    // Update the mouse's current position using encoders or other sensors
}

void Mouse::update_sensors() {
    // Update sensor readings
}

bool Mouse::is_diagonalable() {
    // Implement logic to determine if a diagonal path is possible
    // Check the flood fill values and the presence of walls
    return false; // Placeholder return value
}
propagate the flood values tEach cell updates its neighbors' values if the current cell’s value plus one is less than the neighbor’s value