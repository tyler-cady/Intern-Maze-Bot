#include "MPU6050_6Axis_MotionApps20.h" 
#include <Wire.h>
#include "Kalman.h"

class pid {
private:
  double _max;
  double _min;
  double _Kp;
  double _Kd;
  double _Ki;
  double _pre_error;
  double _integral;
  Kalman _kalman;
  MPU6050_6Axis_MotionApps20& _mpu;
  uint8_t fifoBuffer[64];

public:
  pid(double Kp, double Ki, double Kd, MPU6050_6Axis_MotionApps20& mpu, double max = 255, double min = 0)
    : _max(max), _min(min), _Kp(Kp), _Kd(Kd), _Ki(Ki), _pre_error(0), _integral(0), _kalman(), _mpu(mpu) {}

  double tick(double desired, double dt) {
    // Get the current measurement from the MPU6050
    double current = getMPUData();

    // Use the Kalman filter to smooth the current measurement
    double smoothed_current = _kalman.getAngle(current, 0, dt);

    // Calculate error
    double error = desired - smoothed_current;

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral += error * dt;
    double Iout = _Ki * _integral;
    // Restrict integral to prevent growth after it saturates
    if (_integral > _max) _integral = _max;
    else if (_integral < _min) _integral = _min;

    // Derivative term
    double derivative = (error - _pre_error) / dt;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if (output > _max) output = _max;
    else if (output < _min) output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
  }

  double getMPUData() {
    // Make sure to adapt this method to the functions provided by your library
    if (_mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      Quaternion q;
      _mpu.dmpGetQuaternion(&q, fifoBuffer);
      float euler[3];
      _mpu.dmpGetEuler(euler, &q);
      return euler[0] * 180 / M_PI;  // Return X degrees
    }
    return 0.0;
  }
};

void MPU_setup() {
  MPU6050_6Axis_MotionApps20& mpu;
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock

  Serial.begin(115200);
  Serial.println(F("Initializing I2C devices..."));

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  if (mpu.testConnection()) {
    Serial.println(F("MPU6050 connection successful"));
  } else {
    Serial.println(F("MPU6050 connection failed"));
  }

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize(); // Use the method from the correct library

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
    packetSize = mpu.dmpGetFIFOPacketSize(); // Use the method from the correct library
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}
