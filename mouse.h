#ifndef MOUSE_H
#define MOUSE_H

#include <MPU6050.h>

class Mouse {
public:
    Mouse();
    void turn(float degree);
    void fast_turn(float degree);
    void dash(float distance);
    void diagonal_dash();
    void solve_maze();
    void speedrun();
    int* get_location();
    int* get_valid_turns(int px, int nx, int py, int ny);
    float get_distance(int sensor);
    bool is_diagonalable(); // New function

private:
    void flood_fill();
    void update_position();
    void update_sensors();
    void init_maze();

    int position[2]; // {x, y}
    int maze[9][9]; // Maze representation
    MPU6050 mpu;
    // Add motor and sensor objects here
};

#endif // MOUSE_H