#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include "I2Cdev.h"
#include "NewPing.h"
#include "PinChangeInterrupt.h"

//////////////////////ULTRASONIC VARS/////////////////////////////////////
#define TRIGGER_PIN  17  // Ultrasonic Trigger pin
#define ECHO_FRONT  16 // Ultrasonic Echo pin
#define ECHO_RIGHT  15 // Ultrasonic Echo pin
#define ECHO_LEFT  14 // Ultrasonic Echo pin

NewPing sonar[3] = {
  NewPing(TRIGGER_PIN, ECHO_FRONT, 200),
  NewPing(TRIGGER_PIN, ECHO_RIGHT, 200),
  NewPing(TRIGGER_PIN, ECHO_LEFT, 200)
};

/////////////////////ENCODER MOTOR VARS/////////////////////////////////////
int encoderPin1 = 2; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin2 = 3; //Encoder Otput 'B' must connected with intreput pin of arduino.
volatile int lastEncoded = 0; // Here updated value of encoder store.
volatile long encoderValue = 0; // Raw encoder value
volatile int lastEncoded2 = 0; // Here updated value of encoder store.
volatile long encoderValue2 = 0; // Raw encoder value
const int PWMright1 = 5;
const int PWMright2 = 6;
const int PWMleft1 = 10;
const int PWMleft2 = 11;

#define encoder2Pin1 7
#define encoder2Pin2 8

//////////////// MPU control/status vars///////////////////////////////////
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
float euler[3];         // [psi, theta, phi]    Euler angle container

////////////////////////////COLOR SENSOR/////////////////////////////////
const int s0 = D4;        
const int s1 = D5;  
const int s2 = D6;  
const int s3 = D7;  
const int out = D10; 
int  Red = 0, Blue = 0, Green = 0; 
bool middle = false;
int possible_turns[4] = {1, 1, 1, 1}; //Array to keep track of possible turns. {front, right, left, rear}. 1 = turn possible. 0 = no turn possible

void setup() {
  pinMode(PWMright1, OUTPUT); 
  pinMode(PWMright2, OUTPUT); 
  pinMode(PWMleft1, OUTPUT); 
  pinMode(PWMleft2, OUTPUT);
  Serial.begin(9600); //initialize serial communication

  pinMode(encoderPin1, INPUT_PULLUP); 
  pinMode(encoderPin2, INPUT_PULLUP);

  pinMode(encoder2Pin1, INPUT_PULLUP); 
  pinMode(encoder2Pin2, INPUT_PULLUP);

  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(0, updateEncoder, CHANGE); 
  attachInterrupt(1, updateEncoder, CHANGE);

  attachPCINT(digitalPinToPCINT(encoder2Pin1), updateEncoder2, CHANGE);
  attachPCINT(digitalPinToPCINT(encoder2Pin2), updateEncoder2, CHANGE);

  MPU_setup();
}

void loop() {
  driveToCenter();
  delay(5000); // wait for 5 seconds at the center
  returnToStart();
}

///////////////////////////////////////////////ENCODER FUNCTIONS////////////////////////////////////////////////////
void updateEncoder() {
  int MSB = digitalRead(encoder2Pin1); //MSB = most significant bit
  int LSB = digitalRead(encoder2Pin2); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue--;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue++;

  lastEncoded = encoded; //store this value for next time
}

void updateEncoder2() {
  int MSB = digitalRead(encoder2Pin1); //MSB = most significant bit
  int LSB = digitalRead(encoder2Pin2); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum = (lastEncoded2 << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue2--;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue2++;

  lastEncoded2 = encoded; //store this value for next time
}

/////////////////////////////////////////////MOTORS////////////////////////////////////////////////////
//speed in percentage
void setSpeed(float speed, int pin) {
  float percent = float(speed / 100);
  float dutycycle = float(255 * percent);
  analogWrite(pin, dutycycle); //or digitalwrite????
}

// R_L determines turn right or left. R_L = true : right turn; R_L = false : left turn;
void Turn(bool R_L, int degree, int turn_speed) {
    if (R_L) {
        setSpeed(turn_speed, PWMright1);
        setSpeed(0, PWMright2);
    } else {
        setSpeed(turn_speed, PWMleft1);
        setSpeed(0, PWMleft2);
    }
}

//R_L_BOTH: 0=right motor only; 1=left motor only; 2=both motors
void motors_stop(int R_L_BOTH) {
    if (R_L_BOTH == 0) {
        setSpeed(0, PWMright1);
        setSpeed(0, PWMright2);
    } else if (R_L_BOTH == 1) {
        setSpeed(0, PWMleft1);
        setSpeed(0, PWMleft2);
    } else if (R_L_BOTH == 2) {
        setSpeed(0, PWMright1);
        setSpeed(0, PWMright2);
        setSpeed(0, PWMleft1);
        setSpeed(0, PWMleft2);
    }
}

//direction: true=forward; false=backward
void motors_straight(bool direction, int speed) {
    if (direction) {
        setSpeed(0, PWMright1);
        setSpeed(speed, PWMright2);
        setSpeed(0, PWMleft1);
        setSpeed(speed, PWMleft2);
    } else {
        setSpeed(speed, PWMright1);
        setSpeed(0, PWMright2);
        setSpeed(speed, PWMleft1);
        setSpeed(0, PWMleft2);
    }
}

///////////////////////////////////////////////////////////Ultrasonic/////////////////////////////////////////////////
void updatePossibleTurns() {
    if (sonar[0].ping_cm() < 2) {
        Serial.println("Wall front");
        possible_turns[0] = 0;
    } else {
        Serial.println("No wall front");
    }
    if (sonar[1].ping_cm() < 2) {
        Serial.println("Wall Right");
        possible_turns[1] = 0;
    }
    if (sonar[2].ping_cm() < 2) {
        Serial.println("Wall left");
        possible_turns[2] = 0;
    }
}

void read_ultra(int which, bool recurse) {
    double cm;

    if (which == 0) {
        return;
    }
    if (which == 1) {
        cm = sonar[0].ping_cm();
        Serial.printf("Distance front: %f", cm);   
        Serial.println(" ");
    }
    if (which == 2) {
        cm = sonar[1].ping_cm();
        Serial.printf("Distance right: %f", cm); 
        Serial.println(" ");
    }
    if (which == 3) {
        cm = sonar[2].ping_cm();
        Serial.printf("Distance left: %f", cm); 
        Serial.println(" ");
    }
    delay(500);
    if (recurse) {
        read_ultra(which, recurse);
    }
}

///////////////////////////////////////////////////////////MPU///////////////////////////////////////////////////////
void MPU_setup() {
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock
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

void GetEulerAngles() {
    if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize) {}
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        return    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);

        Serial.print("Yaw: ");
        Serial.print(euler[0] * 180/M_PI);
        Serial.print(" Pitch: ");
        Serial.print(euler[1] * 180/M_PI);
        Serial.print(" Roll: ");
        Serial.println(euler[2] * 180/M_PI);
    }
}

///////////////////////////////////////////////////////////COLOR SENSOR///////////////////////////////////////////////////////
void TCS_Read() {
    digitalWrite(s2, LOW);
    digitalWrite(s3, LOW);
    Red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
    
    digitalWrite(s3, HIGH);
    Blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
    
    digitalWrite(s2, HIGH);
    Green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
    
    if ((Red < Blue) && (Red < Green)) {
        Serial.println("Red Color");
    } else if ((Blue < Red) && (Blue < Green)) {
        Serial.println("Blue Color");
    } else if ((Green < Red) && (Green < Blue)) {
        Serial.println("Green Color");
    }
}

///////////////////////////////////////////////////////////MAZE SOLVING///////////////////////////////////////////////////////
void driveToCenter() {
    // Implementation of driving to the center of the maze
    while (!middle) {
        updatePossibleTurns();
        if (possible_turns[0] == 1) {
            motors_straight(true, 50);  // Move forward
        } else if (possible_turns[1] == 1) {
            Turn(true, 90, 50);  // Turn right
            motors_straight(true, 50);
        } else if (possible_turns[2] == 1) {
            Turn(false, 90, 50);  // Turn left
            motors_straight(true, 50);
        } else {
            motors_stop(2);  // Stop
            middle = true;
        }
        delay(500);
    }
}

void returnToStart() {
    // Implementation of returning to the start of the maze
    while (middle) {
        updatePossibleTurns();
        if (possible_turns[0] == 1) {
            motors_straight(false, 50);  // Move backward
        } else if (possible_turns[1] == 1) {
            Turn(false, 90, 50);  // Turn left
            motors_straight(false, 50);
        } else if (possible_turns[2] == 1) {
            Turn(true, 90, 50);  // Turn right
            motors_straight(false, 50);
        } else {
            motors_stop(2);  // Stop
            middle = false;
        }
        delay(500);
    }
}