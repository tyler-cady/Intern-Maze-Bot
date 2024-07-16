#include <Encoder.h> // Include encoder library
#include <MPU6050.h> // Include MPU6050 library
#include <NewPing.h> // Include NewPing for ultrasonic sensors

// Constants
const int MAZE_WIDTH = 9;
const int MAZE_HEIGHT = 9;
const int INF = 999; // Representing infinity for flood fill
const int CENTER_CELLS[4][2] = {{5, 4}, {6, 4}, {5, 5}, {6, 5}}; // Center coordinates

// Pin definitions
const int TRIG_PIN1 = 2; // Trigger pin for ultrasonic sensor 1
const int ECHO_PIN1 = 3; // Echo pin for ultrasonic sensor 1
const int TRIG_PIN2 = 4; // Trigger pin for ultrasonic sensor 2
const int ECHO_PIN2 = 5; // Echo pin for ultrasonic sensor 2
const int TRIG_PIN3 = 6; // Trigger pin for ultrasonic sensor 3
const int ECHO_PIN3 = 7; // Echo pin for ultrasonic sensor 3
const int LEFT_MOTOR_PIN = 9;
const int RIGHT_MOTOR_PIN = 10;

// Maze grid representation
int maze[MAZE_WIDTH][MAZE_HEIGHT]; // 0 for open, 1 for wall
int floodValues[MAZE_WIDTH][MAZE_HEIGHT]; // Flood fill values
bool visited[MAZE_WIDTH][MAZE_HEIGHT]; // Visited cells

// Position variables
int currentX = 0;
int currentY = 0;

// Motor control
void driveForward(int duration);
void turnLeft();
void turnRight();
void stopMotors();

// Function prototypes
void initializeMaze();
void floodFill();
void findShortestPath();
void driveToCenter();
void returnToStart();
void storeFastestPath();
int getDistance(int sensor);
bool isWall(int x, int y, int direction);

void setup() {
  Serial.begin(9600);
  // Initialize ultrasonic sensors
  NewPing sonar1(TRIG_PIN1, ECHO_PIN1);
  NewPing sonar2(TRIG_PIN2, ECHO_PIN2);
  NewPing sonar3(TRIG_PIN3, ECHO_PIN3);

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
  // Implement flood fill using a queue (BFS)
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
      int newX = x + (dir == 0 ? 0 : dir == 1 ? 0 : dir == 2 ? -1 : 1);
      int newY = y + (dir == 0 ? -1 : dir == 1 ? 1 : dir == 2 ? 0 : 0);

      if (newX >= 0 && newX < MAZE_WIDTH && newY >= 0 && newY < MAZE_HEIGHT &&
          !isWall(x, y, dir) && floodValues[newX][newY] > floodValues[x][y] + 1) {
        floodValues[newX][newY] = floodValues[x][y] + 1;
        queue.push({newX, newY});
      }
    }
  }
}

bool isWall(int x, int y, int direction) {
  // Check if there is a wall in the given direction
  // Implement actual wall checking using sensors or predefined walls
  return (maze[x][y] == 1); // Example implementation
}

void driveToCenter() {
  // Use flood values to navigate to the center
  while (currentX != 5 || currentY != 4) { // Replace with actual center cell logic
    // Logic to choose direction based on flood values
    // Move to the cell with the lowest flood value
    // Update currentX and currentY
  }
}

void returnToStart() {
  // Implement logic to return to the start via a different path
  // Possibly re-run flood fill or use a stored path
}

void driveForward(int duration) {
  // Control motors to drive forward
}

void turnLeft() {
  // Control motors to turn left
}

void turnRight() {
  // Control motors to turn right
}

void stopMotors() {
  // Stop both motors
}

int getDistance(int sensor) {
  // Get distance from the specified ultrasonic sensor
  return 0; // Placeholder for actual distance measurement
}