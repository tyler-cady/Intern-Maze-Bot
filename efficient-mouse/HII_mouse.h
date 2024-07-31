#ifndef HII_MOUSE_H
#define HII_MOUSE_H


#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "NewPing.h"
#include "PinChangeInterrupt.h"
#include <tcs3200.h>
#include "Adafruit_APDS9960.h"
#include "PID_v1.h"




class Mouse {
    public: 
        Mouse();

    private:
        /* Ultrasonic Sensor Vars */   
        static constexpr TRIGGER_PIN  A3
        static constexpr ECHO_FRONT  A0
        static constexpr ECHO_LEFT  A1
        static constexpr ECHO_RIGHT  A2
        static constexpr INTERRUPT_PIN 2
        NewPing sonar[3] = { NewPing(TRIGGER_PIN, ECHO_LEFT, 30), NewPing(TRIGGER_PIN, ECHO_FRONT, 30), NewPing(TRIGGER_PIN, ECHO_RIGHT, 30) };
        float cm[3]; //left,front,right
        bool walls[3]; //left,front, right

        /* Encoder Motor Vars */
        static constexpr encoderPin1 11
        static constexpr encoderPin2 12
        static constexpr encoder2Pin1 7
        static constexpr encoder2Pin2 8
        static constexpr PPR 7
        volatile int lastEncoded = 0;
        volatile long encoderValue = 0;
        volatile int lastEncoded2 = 0;
        volatile long encoderValue2 = 0;
        static constexpr PWMright1 5
        static constexpr PWMright2 6
        static constexpr PWMleft1 9
        static constexpr PWMleft2 10

        /* PID  */
        bool isTurn; // false for straight, true for turning 
        double Input, Output, Setpoint;
        PID myPID; 


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
        void updateEncoders(); // PID version
        int getRotations(bool R_L);
        void resetEncoders();
        void readEncoders();
        void setSpeed(float speed, int pin);
        void turn(bool R_L, int degree, int turn_speed);
        void motors_stop(int R_L_BOTH);
        void motors_straight(bool direction, int speed);
        void read_ultra(int which);
        void checkWalls();
        void APDS_setup();
        void CheckifSolved();
        void MPU_calibrate();
        void MPU_getdata();
        float MPU_getX();
        float MPU_getY();
        float MPU_getZ();
        void MPU_setup();

        /* Constructors */
        
        Mouse::Mouse() 
            : myPID(&Input, &Output, &Setpoint, 2.0, 5.0, 1.0, DIRECT), // Initialize PID with parameters
            encoderValue(0), encoderValue2(0), lastEncoded(0), lastEncoded2(0),
            PPR(400), mode(0), turnDirection(true), turnDegree(0)
        {
            Setpoint = 0; // Set initial setpoint for PID
            myPID.SetMode(AUTOMATIC); // Set PID to automatic mode
        }
};

#endif // MOUSE_H
