#include <Mouse.h> // Include Mouse class
#include <queue> // Include queue for flood fill algorithm

// Constants
const int MAZE_WIDTH = 9;
const int MAZE_HEIGHT = 9;
const int INF = 999; // big # for floodfill
const int CENTER_CELLS[4][2] = {{5, 4}, {6, 4}, {5, 5}, {6, 5}}; // Center coordinates

// Maze grid representation
int maze[MAZE_WIDTH][MAZE_HEIGHT]; // 0 for open, 1 for wall
int floodValues[MAZE_WIDTH][MAZE_HEIGHT]; 
bool visited[MAZE_WIDTH][MAZE_HEIGHT]; // Visited cells

// Position variables
int currentX = 0;
int currentY = 0;
int currentDirection = 0; // 0: North, 1: East, 2: South, 3: West

Mouse mouse; 

// Function prototypes
void initializeMaze();
void floodFill();
void findShortestPath();
void driveToCenter();
void returnToStart();
void storeFastestPath();
bool isWall(int x, int y, int direction);
void updateFloodFill();

void setup() {
  Serial.begin(9600);
  mouse.setup(); // Initialize the mouse

  // Initialize maze and flood values
  initializeMaze();
  floodFill();
  
  // Drive to center and return to start
  driveToCenter();
  returnToStart();
}

void loop() {
  // Execute speed phase if needed
  // driveFastestPath();
}

void initializeMaze() {
  // Set walls and initialize flood values
  for (int x = 0; x < MAZE_WIDTH; x++) {
    for (int y = 0; y < MAZE_HEIGHT; y++) {
      maze[x][y] = 0; // Initialize with no walls
      floodValues[x][y] = INF; // Initialize flood values to infinity
      visited[x][y] = false; // Mark all cells as unvisited
    }
  }
  
  // Set center cells with flood value 0
  for (int i = 0; i < 4; i++) {
    floodValues[CENTER_CELLS[i][0]][CENTER_CELLS[i][1]] = 0;
  }
}

void floodFill() {
  // flood fill (BFS)
  std::queue<std::pair<int, int>> queue;
  
  // Add center cells to the queue
  for (int i = 0; i < 4; i++) {
    queue.push({CENTER_CELLS[i][0], CENTER_CELLS[i][1]});
  }
  
  while (!queue.empty()) {
    auto cell = queue.front();
    queue.pop();
    
    int x = cell.first;
    int y = cell.second;

    // Check neighboring cells (up, down, left, right)
    for (int dir = 0; dir < 4; dir++) {
      int newX = x + (dir == 0 ? 0 : dir == 1 ? 1 : dir == 2 ? 0 : -1);
      int newY = y + (dir == 0 ? -1 : dir == 1 ? 0 : dir == 2 ? 1 : 0);

      if (newX >= 0 && newX < MAZE_WIDTH && newY >= 0 && newY < MAZE_HEIGHT &&
          !isWall(x, y, dir) && floodValues[newX][newY] > floodValues[x][y] + 1) {
        floodValues[newX][newY] = floodValues[x][y] + 1;
        queue.push({newX, newY});
      }
    }
  }
}

bool isWall(int x, int y, int direction) {
  // Check if wall in the given direction
  // Use Mouse class sensors to detect walls
  mouse.checkWalls();
  if (direction == 0) return mouse.walls[0]; // Front
  if (direction == 1) return mouse.walls[1]; // Right
  if (direction == 2) return mouse.walls[2]; // Left
  return false; // Back direction not checked
}

void driveToCenter() {
  while (floodValues[currentX][currentY] != 0) {
    int minVal = INF;
    int nextDirection = -1;
    
    for (int dir = 0; dir < 4; dir++) {
      int newX = currentX + (dir == 0 ? 0 : dir == 1 ? 1 : dir == 2 ? 0 : -1);
      int newY = currentY + (dir == 0 ? -1 : dir == 1 ? 0 : dir == 2 ? 1 : 0);

      if (newX >= 0 && newX < MAZE_WIDTH && newY >= 0 && newY < MAZE_HEIGHT &&
          !isWall(currentX, currentY, dir) && floodValues[newX][newY] < minVal) {
        minVal = floodValues[newX][newY];
        nextDirection = dir;
      }
    }

    if (nextDirection == -1) break; // No valid move, exit
    
    if (nextDirection != currentDirection) {
      if ((currentDirection + 1) % 4 == nextDirection) {
        mouse.Turn(true, 90, 50); // Turn right
      } else if ((currentDirection + 3) % 4 == nextDirection) {
        mouse.Turn(false, 90, 50); // Turn left
      } else {
        mouse.Turn(false, 180, 50); // Turn around
      }
      currentDirection = nextDirection;
    }

    mouse.motors_straight(true, 50); // Drive forward with speed 50
    delay(1000); // Wait for 1 second

    currentX = currentX + (currentDirection == 1 ? 1 : currentDirection == 3 ? -1 : 0);
    currentY = currentY + (currentDirection == 0 ? -1 : currentDirection == 2 ? 1 : 0);

    // Update flood fill based on newly discovered walls
    updateFloodFill();
  }
}

void returnToStart() {
  // Temporarily mark the center as walls to force a different path
  for (int i = 0; i < 4; i++) {
    maze[CENTER_CELLS[i][0]][CENTER_CELLS[i][1]] = 1;
  }

  floodFill(); // Recalculate flood values

  while (currentX != 0 || currentY != 0) {
    int minVal = INF;
    int nextDirection = -1;
    
    for (int dir = 0; dir < 4; dir++) {
      int newX = currentX + (dir == 0 ? 0 : dir == 1 ? 1 : dir == 2 ? 0 : -1);
      int newY = currentY + (dir == 0 ? -1 : dir == 1 ? 0 : dir == 2 ? 1 : 0);

      if (newX >= 0 && newX < MAZE_WIDTH && newY >= 0 && newY < MAZE_HEIGHT &&
          !isWall(currentX, currentY, dir) && floodValues[newX][newY] < minVal) {
        minVal = floodValues[newX][newY];
        nextDirection = dir;
      }
    }

    if (nextDirection == -1) break; // No valid move, exit
    
    if (nextDirection != currentDirection) {
      if ((currentDirection + 1) % 4 == nextDirection) {
        mouse.Turn(true, 90, 50); // Turn right
      } else if ((currentDirection + 3) % 4 == nextDirection) {
        mouse.Turn(false, 90, 50); // Turn left
      } else {
        mouse.Turn(false, 180, 50); // Turn around
      }
      currentDirection = nextDirection;
    }

    mouse.motors_straight(true, 50); // Drive forward with speed 50
    delay(1000); // Wait for 1 second

    currentX = currentX + (currentDirection == 1 ? 1 : currentDirection == 3 ? -1 : 0);
    currentY = currentY + (currentDirection == 0 ? -1 : currentDirection == 2 ? 1 : 0);

    // Update flood fill based on newly discovered walls
    updateFloodFill();
  }

  // Restore the center cells
  for (int i = 0; i < 4; i++) {
    maze[CENTER_CELLS[i][0]][CENTER_CELLS[i][1]] = 0;
  }

  floodFill(); 
}

void updateFloodFill() {

  floodFill();
}
