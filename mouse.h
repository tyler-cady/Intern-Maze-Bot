#ifndef MOUSE_H
#define MOUSE_H


#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "NewPing.h"
#include "PinChangeInterrupt.h"
#include <tcs3200.h>
#include "Adafruit_APDS9960.h"




class Mouse {
    public: 
        Mouse();

    private:
        /* Ultrasonic Sensor Vars */   
        #define TRIGGER_PIN  A3
        #define ECHO_FRONT  A0
        #define ECHO_LEFT  A1
        #define ECHO_RIGHT  A2
        #define INTERRUPT_PIN 2
        NewPing sonar[3] = { NewPing(TRIGGER_PIN, ECHO_LEFT, 30), NewPing(TRIGGER_PIN, ECHO_FRONT, 30), NewPing(TRIGGER_PIN, ECHO_RIGHT, 30) };
        float cm[3]; //left,front,right
        bool walls[3]; //left,front, right

        /* Encoder Motor Vars */
        #define encoderPin1 11
        #define encoderPin2 12
        #define encoder2Pin1 7
        #define encoder2Pin2 8
        #define PPR 7
        volatile int lastEncoded = 0;
        volatile long encoderValue = 0;
        volatile int lastEncoded2 = 0;
        volatile long encoderValue2 = 0;
        #define PWMright1 5
        #define PWMright2 6
        #define PWMleft1 9
        #define PWMleft2 10

        /* MPU control & status vars */
        MPU6050 mpu;
        bool dmpReady = false;
        uint8_t mpuIntStatus;
        uint8_t devStatus;
        uint16_t packetSize;
        uint16_t fifoCount;
        uint8_t fifoBuffer[64];
        Quaternion q;
        float euler[3];

        /* Color Sensor */
        #define ADPS_I2C 0x39
        Adafruit_APDS9960 apds;
        uint16_t r, g, b, c;

        /* Functions */
        void updateEncoder();
        void updateEncoder2();
        int getRotations(bool R_L);
        void resetEncoders();
        void readEncoders();
        void setSpeed(float speed, int pin);
        void Turn(bool R_L, int degree, int turn_speed);
        void motors_stop(int R_L_BOTH);
        void motors_straight(bool direction, int speed);
        void read_ultra(int which, bool recurse);
        void checkWalls();
        void APDS_setup();
        void APDS_GetColors();
        void CheckifSolved();
        void MPU_getdata();
        float MPU_getX();
        float MPU_getY();
        float MPU_getZ();
        void MPU_setup();
};

#endif // MOUSE_H
