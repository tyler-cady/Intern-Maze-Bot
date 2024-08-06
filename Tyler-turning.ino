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
    Serial.println(target_heading);
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

// void moveForward(bool forward) {
 
//     mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
//     mpu.dmpGetQuaternion(&q, fifoBuffer);
//     mpu.dmpGetGravity(&gravity, &q);
//     mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//     float target_heading = forward ? ypr[0] + M_PI_2 : ypr[0] - M_PI_2;
 
//     while (true){
 
//         mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
//         mpu.dmpGetQuaternion(&q, fifoBuffer);
//         mpu.dmpGetGravity(&gravity, &q);
//         mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
 
//         float error = target_heading - ypr[0];
//         float correction = rightMotorPID.tick(0, error, -0.2399); // assuming dt is 0.1s
//         if (forward) {
//             analogWrite(PWMright2, init_speed_right);
//             analogWrite(PWMleft2, init_speed_left);
//             analogWrite(PWMleft1, 0);
//             analogWrite(PWMright1, 0);      
//         } else {
//             analogWrite(PWMleft1, init_speed_left);
//             analogWrite(PWMright1, init_speed_right);
//             analogWrite(PWMright2, 0);
//             analogWrite(PWMleft2, 0);
//         }
//         delay(1000); // move forward for 1 block (adjust timing as needed)
//     }
//     // Stop the motors after turning
//     analogWrite(PWMleft1, 0);
//     analogWrite(PWMleft2, 0);
//     analogWrite(PWMright1, 0);
//     analogWrite(PWMright2, 0);
// }

void MPU_setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();   
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
	
    while (Serial.available() && Serial.read()); // empty buffer

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(26);
    mpu.setYGyroOffset(-48);
    mpu.setZGyroOffset(-17);
    mpu.setXAccelOffset(-1602);
    mpu.setYAccelOffset(713);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        // Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        // Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        // Serial.println(F(")..."));
        // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();

        Serial.print(F("Hold the mouse still to true North stationary values..."));
        delay(3000);
        Serial.print(F("Straight is: "));
        MPU_0_straight = MPU_getX();
        delay(1000);

    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
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
void MPU_getdata(){
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
  // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("ypr\t");
      // Serial.print(ypr[0] * 180/M_PI);
      // Serial.print("\t");
      // Serial.print(ypr[1] * 180/M_PI);
      // Serial.print("\t");
      Serial.println(ypr[2] * 180/M_PI);
  }
}
//return X degrees
float MPU_getX(){
  MPU_getdata();
  float X_degrees=(ypr[2] * 180/M_PI);
  return 2*X_degrees;
}

void loop() {
    // turn(true);
    Serial.println(MPU_getX());
    delay(1000);
}
