#include "Flood.h"
#include <stdlib.h>

bool isValidMove(int x, int y, int maze[MAZE_SIZE][MAZE_SIZE]) {
    return (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE && maze[x][y] == 0);
}

Stack* createStack(int capacity) {
    Stack* stack = (Stack*)malloc(sizeof(Stack));
    stack->capacity = capacity;
    stack->top = -1;
    stack->cells = (Cell*)malloc(stack->capacity * sizeof(Cell));
    return stack;
}

void freeStack(Stack* stack) {
    free(stack->cells);
    free(stack);
}

bool isStackFull(Stack* stack) {
    return stack->top == stack->capacity - 1;
}

bool isStackEmpty(Stack* stack) {
    return stack->top == -1;
}

void push(Stack* stack, Cell item) {
    if (isStackFull(stack))
        return;
    stack->cells[++stack->top] = item;
}

Cell pop(Stack* stack) {
    if (isStackEmpty(stack)) {
        Cell empty = {-1, -1, -1}; // Return an invalid cell
        return empty;
    }
    return stack->cells[stack->top--];
}

void floodFill(int maze[MAZE_SIZE][MAZE_SIZE], Cell start, Cell end) {
    Stack* stack = createStack(MAZE_SIZE * MAZE_SIZE);
    push(stack, start);

    while (!isStackEmpty(stack)) {
        Cell current = pop(stack);

        if (current.x == end.x && current.y == end.y) {
            break;
        }

        if (isValidMove(current.x + 1, current.y, maze)) {
            push(stack, {current.x + 1, current.y, current.distance + 1});
        }
        if (isValidMove(current.x - 1, current.y, maze)) {
            push(stack, {current.x - 1, current.y, current.distance + 1});
        }
        if (isValidMove(current.x, current.y + 1, maze)) {
            push(stack, {current.x, current.y + 1, current.distance + 1});
        }
        if (isValidMove(current.x, current.y - 1, maze)) {
            push(stack, {current.x, current.y - 1, current.distance + 1});
        }
    }

    freeStack(stack);
}