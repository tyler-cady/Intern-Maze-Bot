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
    bool updated;
    do {
        updated = false;
        for (int y = 0; y < 9; ++y) {
            for (int x = 0; x < 9; ++x) {
                if (maze[y][x] != -1) {
                    int min_dist = 1000;
                    if (y > 0 && maze[y - 1][x] != -1) min_dist = min(min_dist, maze[y - 1][x] + 1);
                    if (y < 8 && maze[y + 1][x] != -1) min_dist = min(min_dist, maze[y + 1][x] + 1);
                    if (x > 0 && maze[y][x - 1] != -1) min_dist = min(min_dist, maze[y][x - 1] + 1);
                    if (x < 8 && maze[y][x + 1] != -1) min_dist = min(min_dist, maze[y][x + 1] + 1);

                    if (min_dist < maze[y][x]) {
                        maze[y][x] = min_dist;
                        updated = true;
                    }
                }
            }
        }
    } while (updated);
}

void Mouse::init_maze() {
    for (int y = 0; y < 9; ++y) {
        for (int x = 0; x < 9; ++x) {
            maze[y][x] = 1000; // Start with a large number
        }
    }
    maze[4][3] = 0;
    maze[4][4] = 0;
    maze[5][3] = 0;
    maze[5][4] = 0;
}

void Mouse::update_position() {
    // Update the mouse position based on encoder readings
}

void Mouse::update_sensors() {
    // Update sensor readings
}

bool Mouse::is_diagonalable() {
    int x = position[0];
    int y = position[1];

    // Check for available paths that can be bypassed by diagonal movement
    // Assuming 0 is free space and -1 is wall in the maze representation
    bool can_move = true;

    // Check the four potential corners for diagonal movement
    if (x < 8 && y < 8 && maze[y + 1][x + 1] == 0) {
        // Check if moving down-right is clear
        if (maze[y][x + 1] == -1 || maze[y + 1][x] == -1) can_move = false;
    }
    if (x > 0 && y < 8 && maze[y + 1][x - 1] == 0) {
        // Check if moving down-left is clear
        if (maze[y][x - 1] == -1 || maze[y + 1][x] == -1) can_move = false;
    }
    if (x < 8 && y > 0 && maze[y - 1][x + 1] == 0) {
        // Check if moving up-right is clear
        if (maze[y][x + 1] == -1 || maze[y - 1][x] == -1) can_move = false;
    }
    if (x > 0 && y > 0 && maze[y - 1][x - 1] == 0) {
        // Check if moving up-left is clear
        if (maze[y][x - 1] == -1 || maze[y - 1][x] == -1) can_move = false;
    }

    return can_move;
}