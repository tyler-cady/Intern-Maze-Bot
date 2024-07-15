#include "Mouse.h"
#include <Wire.h>

// Create a Mouse object
Mouse micromouse;

void setup() {
    // Initialize the serial communication for debugging
    Serial.begin(9600);
    
    // Initialize the Mouse object
    micromouse = Mouse();

    // Optionally, add other initialization code here
    // Example: setting up pin modes for motors or sensors
}

void loop() {
    // Run the speedrun function to solve the maze as quickly as possible
    micromouse.speedrun();

    // Optionally, add a delay or other logic here
    // Example: delay(1000); // Wait for a second before the next loop iteration
}