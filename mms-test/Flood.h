#ifndef FLOOD_H
#define FLOOD_H

#define MAZE_SIZE 9

struct Cell {
    int x, y;
    int distance; 
};

bool isValidMove(int x, int y, int maze[MAZE_SIZE][MAZE_SIZE]);

struct Stack{
    Cell* cells;
    int top;
    int capacity;
};

Stack* createStack(int capacity);
void freeStack(Stack* stack);
bool isStackFUll(Stack* stack);
bool isStackEmpty(Stack* stack);
void push(Stack* stack);
Cell pop(Stack* stack);

void floodFill(int maze[MAZE_SIZE][MAZE_SIZE], Cell start, Cell end);
#endif // Flood.h