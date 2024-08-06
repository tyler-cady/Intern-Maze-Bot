#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "NewPing.h"
#include "PinChangeInterrupt.h"
#include <tcs3200.h>
#include "Adafruit_APDS9960.h"
#include "pid.h"

// Ultrasonic Variables
#define TRIGGER_PIN A3
#define ECHO_FRONT A1
#define ECHO_LEFT A0
#define ECHO_RIGHT A2
#define INTERRUPT_PIN 2
NewPing sonar[3] = { NewPing(TRIGGER_PIN, ECHO_LEFT, 30), NewPing(TRIGGER_PIN, ECHO_FRONT, 500), NewPing(TRIGGER_PIN, ECHO_RIGHT, 30) };
float cm[3]; // left, front, right
bool walls[3]; // left, front, right

// Encoder Motor Variables
#define encoderPin1 11
#define encoderPin2 12
#define encoder2Pin1 7
#define encoder2Pin2 8
#define PPR 7
volatile int lastEncoded = 0;
volatile long encoderValue = 0;
volatile int lastEncoded2 = 0;
volatile long encoderValue2 = 0;
#define PWMright1 9
#define PWMright2 10
#define PWMleft1 5
#define PWMleft2 6
float speed_right, speed_left;
float init_speed_right = 83;
float init_speed_left = 96;
float change_of_speed = 5;
int distance_to_stop = 6;
// pid leftMotorPID(0.1, 0.3, .05, 255, 0);
// pid rightMotorPID(0.1, 0.3, .05, 255, 0);

pid leftMotorPID(109.553, -0.4799, -0.2399, 0);
pid rightMotorPID(109.553, -0.4799, -0.2399, 0);

// MPU control/status variables
MPU6050 mpu;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
float euler[3];
VectorFloat gravity;
float ypr[3];
float MPU_90_left, MPU_90_right, MPU_0_straight, needle;

// Ultrasonic Functions
void read_right_ultra() {
    cm[2] = sonar[2].ping_cm();
}

void read_front_ultra() {
    cm[1] = sonar[1].ping_cm();
}

void read_left_ultra() {
    cm[0] = sonar[0].ping_cm();
}

void pid_laneKeep() {
    read_left_ultra();
    read_right_ultra();
    float left_distance = cm[0];
    float right_distance = cm[2];
    float error = left_distance - right_distance;
    float correction = leftMotorPID.tick(0, error, 0.1); // assuming desired position is 0 (centered) and dt is 0.1s
    speed_left += correction;
    speed_right -= correction;
    analogWrite(PWMleft1, speed_left);
    analogWrite(PWMright1, speed_right);
}

void turn(bool isRight, double init_heading = 0) {
    mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    float target_heading = isRight ? ypr[0] + M_PI_2 : ypr[0] - M_PI_2;

    while (abs(ypr[0] - target_heading) > 0.01) {
        mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        float error = target_heading - ypr[0];
        float correction = rightMotorPID.tick(0, error, 0.1); // assuming dt is 0.1s

        if (isRight) {
            analogWrite(PWMleft1, init_speed_left + correction);
            analogWrite(PWMleft2, 0); // Stop the other pin for left motor
            analogWrite(PWMright2, init_speed_right + correction); // Adjusted for opposite wiring
            analogWrite(PWMright1, 0); // Stop the other pin for right motor
        } else {
            analogWrite(PWMleft2, init_speed_left - correction); // Adjusted for opposite wiring
            analogWrite(PWMleft1, 0); // Stop the other pin for left motor
            analogWrite(PWMright1, init_speed_right - correction);
            analogWrite(PWMright2, 0); // Stop the other pin for right motor
        }

        delay(10);
    }

    // Stop the motors after turning
    analogWrite(PWMleft1, 0);
    analogWrite(PWMleft2, 0);
    analogWrite(PWMright1, 0);
    analogWrite(PWMright2, 0);
}

void moveForward(bool forward) {
 
    mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    float target_heading = isRight ? ypr[0] + M_PI_2 : ypr[0] - M_PI_2;
 
    while (abs(ypr[0] - target_heading) > 0.01){
 
        mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
 
        float error = target_heading - ypr[0];
        float correction = rightMotorPID.tick(0, error, -0.2399); // assuming dt is 0.1s
        if forward {
            analogWrite(PWMright2, init_speed_right);
            analogWrite(PWMleft2, init_speed_left);
            analogWrite(PWMleft1, 0);
            analogWrite(PWMright1, 0);      
        } else {
            analogWrite(PWMleft1, init_speed_left);
            analogWrite(PWMright1, init_speed_right);
            analogWrite(PWMright2, 0);
            analogWrite(PWMleft2, 0);
        }
        delay(1000); // move forward for 1 block (adjust timing as needed)
    }
    // Stop the motors after turning
    analogWrite(PWMleft1, 0);
    analogWrite(PWMleft2, 0);
    analogWrite(PWMright1, 0);
    analogWrite(PWMright2, 0);
}

void MPU_setup() {
    Wire.begin();
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void updateEncoder() {
    int MSB = digitalRead(encoderPin1); // MSB = most significant bit
    int LSB = digitalRead(encoderPin2); // LSB = least significant bit

    int encoded = (MSB << 1) | LSB; // converting the 2 pin value to single number
    int sum  = (lastEncoded << 2) | encoded; // adding it to the previous encoded value

    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;

    lastEncoded = encoded; // store this value for next time
}

void updateEncoder2() {
    int MSB = digitalRead(encoder2Pin1); // MSB = most significant bit
    int LSB = digitalRead(encoder2Pin2); // LSB = least significant bit

    int encoded = (MSB << 1) | LSB; // converting the 2 pin value to single number
    int sum  = (lastEncoded2 << 2) | encoded; // adding it to the previous encoded value

    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue2 --;
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue2 ++;

    lastEncoded2 = encoded; // store this value for next time
}

void setup() {
    pinMode(PWMright1, OUTPUT);
    pinMode(PWMright2, OUTPUT);
    pinMode(PWMleft1, OUTPUT);
    pinMode(PWMleft2, OUTPUT);
    Serial.begin(9600);
    pinMode(encoderPin1, INPUT_PULLUP);
    pinMode(encoderPin2, INPUT_PULLUP);
    pinMode(encoder2Pin1, INPUT_PULLUP);
    pinMode(encoder2Pin2, INPUT_PULLUP);
    attachPCINT(digitalPinToPCINT(encoderPin1), updateEncoder, CHANGE);
    attachPCINT(digitalPinToPCINT(encoderPin2), updateEncoder, CHANGE);
    attachPCINT(digitalPinToPCINT(encoder2Pin1), updateEncoder2, CHANGE);
    attachPCINT(digitalPinToPCINT(encoder2Pin2), updateEncoder2, CHANGE);
    MPU_setup();
    speed_right = init_speed_right;
    speed_left = init_speed_left;
    delay(2000);
}

void loop() {
    turn(true);
    delay(1000);
}
