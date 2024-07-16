#include <Wire.h>
#include <MPU6050.h>  // Include your custom MPU6050 processor header
#include <Encoder.h>

// Define MPU6050 connections
MPU6050 mpu;

// Define encoder connections
Encoder encoderLeft(2, 3);   //  pin numbers for encoder A and B channels
Encoder encoderRight(4, 5);  //  pin numbers for encoder A and B channels

// Motor control pins
const int motorPin1 = 6;  // Motor 1 direction pin
const int motorPin2 = 7;  // Motor 1 PWM speed control pin
const int motorPin3 = 8;  // Motor 2 direction pin
const int motorPin4 = 9;  // Motor 2 PWM speed control pin

// PID constants (adjust as needed)
double Kp = 1.0;
double Ki = 0.0;
double Kd = 0.0;

// Variables for PID control
double error = 0;
double lastError = 0;
double integral = 0;
double derivative = 0;

// Target velocity and turn angle (adjust as needed)
double targetVelocity = 0.0;

// Motor speeds
int motorSpeed1 = 0;
int motorSpeed2 = 0;

void setup() {
  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();

  // Initialize motor control pins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);

  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  // Read MPU6050 data
  uint8_t raw_buffer[14];
  Wire.requestFrom(0x68, 14);
  int i = 0;
  while (Wire.available()) {
    raw_buffer[i++] = Wire.read();
  }

  // Process MPU6050 data using custom processor
  custom_msgs::EulerMotion motionData = processMpu6050Data(raw_buffer);

  // Calculate turn angle
  double turnAngle = motionData.yaw;  // Assuming yaw angle is used for turning

  // Read encoder values
  long encoderLeftValue = encoderLeft.read();
  long encoderRightValue = encoderRight.read();

  // Calculate velocities based on encoder readings
  double currentVelocityLeft = encoderLeftValue * 0.1;  // Conversion factor
  double currentVelocityRight = encoderRightValue * 0.1;  // Conversion factor

  // Calculate error for PID control
  error = targetVelocity - (currentVelocityLeft + currentVelocityRight) / 2.0;
  integral += error;
  derivative = error - lastError;
  lastError = error;

  // Calculate motor speeds using PID control
  motorSpeed1 = Kp * error + Ki * integral + Kd * derivative;
  motorSpeed2 = Kp * error + Ki * integral + Kd * derivative;

  // Adjust motor speeds for turning using MPU6050 data
  // : Adjust motor speeds based on turn angle
  motorSpeed1 += turnAngle * 10;  //  adjustment factor
  motorSpeed2 -= turnAngle * 10;  //  adjustment factor

  // Ensure motor speeds are within valid range (-255 to 255)
  motorSpeed1 = constrain(motorSpeed1, -255, 255);
  motorSpeed2 = constrain(motorSpeed2, -255, 255);

  // Set motor directions based on motor speeds
  if (motorSpeed1 >= 0) {
    digitalWrite(motorPin1, HIGH);
    analogWrite(motorPin2, motorSpeed1);
  } else {
    digitalWrite(motorPin1, LOW);
    analogWrite(motorPin2, -motorSpeed1);
  }

  if (motorSpeed2 >= 0) {
    digitalWrite(motorPin3, HIGH);
    analogWrite(motorPin4, motorSpeed2);
  } else {
    digitalWrite(motorPin3, LOW);
    analogWrite(motorPin4, -motorSpeed2);
  }

  // Print debug information
  Serial.print("Turn Angle: ");
  Serial.print(turnAngle);
  Serial.print("\tLeft Velocity: ");
  Serial.print(currentVelocityLeft);
  Serial.print("\tRight Velocity: ");
  Serial.print(currentVelocityRight);
  Serial.print("\tMotor Speed 1: ");
  Serial.print(motorSpeed1);
  Serial.print("\tMotor Speed 2: ");
  Serial.println(motorSpeed2);

  // Delay to control loop rate (adjust as needed)
  delay(100);
}

void move(bool pX, bool nX, bool pY, bool nY) {
  // Adjust motor speeds based on movement flags
  if (pX) {
    // Perform a 90-degree right turn
    motorSpeed1 = 100;  // Adjust speed for right turn
    motorSpeed2 = -100; // Adjust speed for right turn
  } else if (nX) {
    // Perform a 90-degree left turn
    motorSpeed1 = -100; // Adjust speed for left turn
    motorSpeed2 = 100;  // Adjust speed for left turn
  } else if (pY) {
    // Move forward
    motorSpeed1 = 100;  // Adjust speed for forward movement
    motorSpeed2 = 100;  // Adjust speed for forward movement
  } else if (nY) {
    // Move backward
    motorSpeed1 = -100; // Adjust speed for backward movement
    motorSpeed2 = -100; // Adjust speed for backward movement
  }

  // Set motor directions based on motor speeds
  if (motorSpeed1 >= 0) {
    digitalWrite(motorPin1, HIGH);
    analogWrite(motorPin2, motorSpeed1);
  } else {
    digitalWrite(motorPin1, LOW);
    analogWrite(motorPin2, -motorSpeed1);
  }

  if (motorSpeed2 >= 0) {
    digitalWrite(motorPin3, HIGH);
    analogWrite(motorPin4, motorSpeed2);
  } else {
    digitalWrite(motorPin3, LOW);
    analogWrite(motorPin4, -motorSpeed2);
  }
}
