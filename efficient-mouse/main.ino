#include "pid.hpp"
#include "MPU6050.h"
#include <Wire.h>

MPU6050 mpu;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
float euler[3];

void MPU_setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();   
        Wire.setClock(400000); // 400kHz I2C clock
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    while (!Serial);
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    while (Serial.available() && Serial.read());

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(26);
    mpu.setYGyroOffset(-48);
    mpu.setZGyroOffset(-17);
    mpu.setXAccelOffset(-1602);
    mpu.setYAccelOffset(713);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

pid leftMotorPID(1.0, 0.5, 0.1, mpu, 100, 0);
pid rightMotorPID(1.0, 0.5, 0.1, mpu, 100, 0);

void updateMotorSpeeds(double desiredSpeed, double dt) {
  double leftMotorOutput = leftMotorPID.tick(desiredSpeed, dt);
  double rightMotorOutput = rightMotorPID.tick(desiredSpeed, dt);

  // Tests 
  Serial.println("PID OUTPUTS R/L:");
  Serial.println(rightMotorOutput);
  Serial.println(leftMotorOutput);

  setSpeed(0, PWMleft1);
  setSpeed(leftMotorOutput, PWMleft2);
  setSpeed(0, PWMright1);
  setSpeed(rightMotorOutput, PWMright2);
}

void setup() {
  Serial.begin(115200);
  MPU_setup();
}

void loop() {
  // Update motor speeds with desired speed and delta time
  double desiredSpeed = 50.0; // Example desired speed
  double dt = 0.01; // Example delta time, 10ms
  updateMotorSpeeds(desiredSpeed, dt);
  delay(10); // Delay to simulate real-time loop
}