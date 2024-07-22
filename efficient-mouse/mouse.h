#ifndef MOUSE_H
#define MOUSE_H

#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>
#include <APDS9960.h>
#include <MPU6050.h>
#include <NewPing.h>

class Mouse {
public:
    Mouse();

    void setup();
    void updateEncoders();
    int getRotations(bool R_L) const;
    void resetEncoders();
    void readEncoders() const;
    void setSpeed(float speed, int pin) const;
    void Turn(bool R_L, int degree, int turn_speed);
    void motors_stop(int R_L_BOTH) const;
    void motors_straight(bool direction, int speed);
    float read_ultra(int which);
    void checkWalls();
    void APDS_setup();
    void CheckifSolved();
    void MPU_getdata();
    float MPU_getX() const;
    float MPU_getY() const;
    float MPU_getZ() const;
    void MPU_setup();

private:
    static constexpr int PWM_PINS[4] = {PWMright1, PWMright2, PWMleft1, PWMleft2};
    static constexpr int ENCODER_PINS1[2] = {encoderPin1, encoderPin2};
    static constexpr int ENCODER_PINS2[2] = {encoder2Pin1, encoder2Pin2};

    static void updateEncoder1();
    static void updateEncoder2();
    static void updateEncoder(bool isEncoder1);

    static volatile int encoderValue1;
    static volatile int encoderValue2;
    static volatile int lastEncoded1;
    static volatile int lastEncoded2;

    volatile bool isTurn;
    volatile bool R_L;
    double Input, Output, Setpoint;
    PID myPID;

    APDS9960 apds;
    MPU6050 mpu;
    NewPing sonar[3];
    int cm[3];
    bool walls[3];
    int r, g, b, c;
    bool dmpReady = false;
    uint8_t mpuIntStatus, devStatus;
    uint16_t packetSize;
    uint8_t fifoBuffer[64];
    Quaternion q;
    float euler[3];
};

#endif
