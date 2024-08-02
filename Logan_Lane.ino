#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "NewPing.h"
#include "PinChangeInterrupt.h"
#include <tcs3200.h>
#include "Adafruit_APDS9960.h"
#include "pid.h"

//////////////////////ULTRASONIC VARS/////////////////////////////////////
#define TRIGGER_PIN  A3  // Ultrasonic Trigger pin
#define ECHO_FRONT  A1 // Ultrasonic Echo pin
#define ECHO_LEFT  A0 // Ultrasonic Echo pin
#define ECHO_RIGHT  A2 // Ultrasonic Echo pin

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards. CHANGE LATER

NewPing sonar[3] = {
  NewPing(TRIGGER_PIN, ECHO_LEFT,30),
  NewPing(TRIGGER_PIN, ECHO_FRONT,500),
  NewPing(TRIGGER_PIN, ECHO_RIGHT,30)
};


float cm[3]; //left,front,right
bool walls[3]; //left,front, right
/////////////////////ENCODER MOTOR VARS/////////////////////////////////////
#define encoderPin1 11 //Encoder Output 'A' must connected with intreput pin of arduino.
#define encoderPin2 12 //Encoder Otput 'B' must connected with intreput pin of arduino.
#define encoder2Pin1 7
#define encoder2Pin2 8
#define PPR 7

volatile int lastEncoded = 0; // Here updated value of encoder store.
volatile long encoderValue = 0; // Raw encoder value
volatile int lastEncoded2 = 0; // Here updated value of encoder store.
volatile long encoderValue2 = 0; // Raw encoder value

#define PWMright1 9
#define PWMright2 10
#define PWMleft1 5
#define PWMleft2 6

float speed_right, speed_left;
float init_speed_right = 83; //83 //63
float init_speed_left = 96; //96 //76
float change_of_speed = 5; //5
int distance_to_stop = 6;

pid leftMotorPID(0.1, 0.3, .05, 255, 0);
pid rightMotorPID(0.1, 0.3, .05, 255, 0);
//////////////// MPU control/status vars///////////////////////////////////
MPU6050 mpu;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
float euler[3];         // [psi, theta, phi]    Euler angle container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float MPU_90_left, MPU_90_right, MPU_0_straight,needle;

////////////////////////////COLOR SENSOR/////////////////////////////////
#define ADPS_I2C 0x39

Adafruit_APDS9960 apds;
uint16_t r, g, b, c;

bool middle = false;

void setup() {

  pinMode(PWMright1, OUTPUT); 
  pinMode(PWMright2, OUTPUT); 
  pinMode(PWMleft1, OUTPUT); 
  pinMode(PWMleft2, OUTPUT);
  Serial.begin(9600); //initialize serial comunication

  pinMode(encoderPin1, INPUT_PULLUP); 
  pinMode(encoderPin2, INPUT_PULLUP);

  pinMode(encoder2Pin1, INPUT_PULLUP); 
  pinMode(encoder2Pin2, INPUT_PULLUP);

  // digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  // digitalWrite(encoderPin2, HIGH); //turn pullup resistor on

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  // attachInterrupt(0, updateEncoder, CHANGE); 
  // attachInterrupt(1, updateEncoder, CHANGE);
  attachPCINT(digitalPinToPCINT(encoderPin1),updateEncoder, CHANGE);
  attachPCINT(digitalPinToPCINT(encoderPin2),updateEncoder, CHANGE);
  attachPCINT(digitalPinToPCINT(encoder2Pin1),updateEncoder2, CHANGE);
  attachPCINT(digitalPinToPCINT(encoder2Pin2),updateEncoder2, CHANGE);


  

  // MPU_setup();
  // APDS_setup();
  speed_right=66;
  speed_left= 76;
  Serial.println("straight");
  // motors_straight(true,50,50,50);
  // motors_straight(true,speed_right/2,speed_left/2,25); //difference of 42.5,49 or 
  delay(200);

}
///WORKS WITH OFFSET R=83 and L=96
unsigned long starttime = millis();

void loop() {
// APDS_GetColors();  
// delay(50);
// // CheckifSolved();
// delay(50);
// read_ultra(3,true);
//  unsigned long looptime = millis();
//  if(looptime-starttime >=1500){
// delay(50);
delay(250);
read_front_ultra();
while(cm[1]> 6 || cm[1] == 0){
forward_1_block();
}
delay(1000);
turn_90(false);
delay(1000);
while(cm[1]> 5 || cm[1] == 0){
forward_1_block();
}
delay(1000);
turn_90(true);
delay(1000);
forward_1_block();
delay(5000);

// turn_90(true);
// delay(2000);
// turn_90(false);

} 
bool first_time;
void forward_1_block(){
  delay(1000);
  unsigned long start= millis();
  unsigned long current = millis();
  first_time = true;
  while(current-start <= 800){ //800
  Logan_lanekeep();
  current = millis();
  Serial.println("current millis: ");
  Serial.println(current-start);
 }
 Serial.print("out of loop");
  // motors_straight(true,83/2,92/2,25);
  // delay(250);
  // motors_straight(true,83,92,25);

  motors_stop(2);
}

void turn_90(bool R_L){
  motors_stop(2);
  delay(500);
  Turn(R_L,90,83);
  delay(350);
  motors_stop(2);
}
void Logan_lanekeep(){
// delay(100);
// read_ultra(3,true);
read_right_ultra();
delay(50);
read_left_ultra();
delay(50);
read_front_ultra();
delay(50);
  speed_right=init_speed_right;
  speed_left= init_speed_left;
if((cm[1] < 10 && cm[1] !=0)){
  Serial.println("STOP FROM INSIDE.");
  motors_stop(2);
  return;
}
if(cm[0] == cm[2]){
  Serial.print("good");
  speed_right=init_speed_right;
  speed_left =init_speed_left;
}
else if(cm[0] > cm[2]){//(cm[0] < 2 && cm[2] > 2 ){ //left
  Serial.println("Turning left");
  speed_right=speed_right+change_of_speed;
  speed_left=speed_left-change_of_speed;
}
else if (cm[2] > cm[0]){ //(cm[2] < 2 && cm[0] > 2 ){ //cm[2] = right cm[0] = left
    Serial.println("Turning right");
    speed_right=speed_right-change_of_speed;
    speed_left=speed_left+change_of_speed;  
}
if(first_time){
  motors_straight(true,speed_right/2,speed_left/2,25);
  delay(50);
  first_time = false;
}
  motors_straight(true,speed_right,speed_left,25);
  // delay(50);
}


///////////////////////////////////////////////ENCODER FUNCTIONS////////////////////////////////////////////////////
void updateEncoder(){
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;

  lastEncoded = encoded; //store this value for next time
}
void updateEncoder2(){
  int MSB = digitalRead(encoder2Pin1); //MSB = most significant bit
  int LSB = digitalRead(encoder2Pin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded2 << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue2 --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue2 ++;

  lastEncoded2 = encoded; //store this value for next time
}
//R_L: true right(1), false left(2)
int getRotations(bool R_L){
  if(R_L){
    return int(encoderValue/PPR);  //divide by Pulses per rotation to get total number of rotations recorded
  } else{
    return int(encoderValue2/PPR); //divide by Pulses per rotation to get total number of rotations recorded
  }

}
//to be used after a turn is made to ensure proper count
void resetEncoders(){
  encoderValue=0;
  encoderValue2=0;
}

//GET RID OF AFTER TESTING FOR OUPTUT PURPOSES ONLY
void readEncoders(){
 Serial.print("Encoder1: ");
 Serial.print(encoderValue, DEC);
 Serial.println(" ");
 Serial.print("Encoder2: ");
 Serial.print(encoderValue2, DEC);
 Serial.println(" "); 
}
/////////////////////////////////////////////MOTORS////////////////////////////////////////////////////
//speed from 0 t0 255
void setSpeed(float speed, int pin){
  // analogWriteResolution(8);
  if(speed >255 ){
    speed=255;    
  }
  analogWrite(pin, speed); //send PWM to H bridge
}

// R_L determines turn right or left. R_L = true : right turn; R_L = false : left turn; turn_speed hardcoded for now
void Turn(bool R_L, int degree, int turn_speed) 
{
        if (R_L){
            setSpeed(83,PWMright1);
            setSpeed(0,PWMright2);
            setSpeed(0,PWMleft1);
            setSpeed(96,PWMleft2);
        }
        else{
            setSpeed(0,PWMright1);
            setSpeed(83,PWMright2);
            setSpeed(96,PWMleft1);
            setSpeed(0,PWMleft2);
        }
    resetEncoders();  //reset encoders to count for next turn
}
//R_L_BOTH: 0=right motor only; 1=left motor only; 2=both motors
void motors_stop(int R_L_BOTH) 
{
    if (R_L_BOTH == 0){ //stop Right motor
        setSpeed(0,PWMright1);
        setSpeed(0,PWMright2);
    }
    else if (R_L_BOTH == 1){ //stop left motor
        setSpeed(0,PWMleft1);
        setSpeed(0,PWMleft2);
    }
    else if (R_L_BOTH == 2){ //stop both motors

        setSpeed(0,PWMright1);
        setSpeed(0,PWMright2);
        setSpeed(0,PWMleft1);
        setSpeed(0,PWMleft2);
        // delay(50);

    }
}
//direction: true=forward; false=backwards. NEED TO BE CALIBRATED BASED ON ORIENTATION OF MOTORS IN CHASSIS
void motors_straight(bool direction, float speed_right, float speed_left, int delay_right) 
{
    if (direction){ //forward direction
        setSpeed(0,PWMleft1);
        setSpeed(speed_left,PWMleft2);
        delay(delay_right);
        setSpeed(speed_right,PWMright2);
        setSpeed(0,PWMright1);
    }
    else{ //backwards
        setSpeed(speed_right,PWMright1);
        setSpeed(0,PWMright2);
        setSpeed(speed_left,PWMleft1);
        setSpeed(0,PWMleft2);
    }
}
void read_right_ultra(){
    cm[2] = sonar[2].ping_cm(30) ;

}
void read_front_ultra(){
    cm[1] = sonar[1].ping_cm(30) ;
         Serial.print("Distance front: ");
        Serial.print(cm[1]);
        Serial.println(" ");
}
void read_left_ultra(){
    cm[0] = sonar[0].ping_cm(30) ;

}
///////////////////////////////////////////////////////////Ultrasonic/////////////////////////////////////////////////
//prints each sensor's distance depending. which => 0 = none; 1 = front sensor; 2 =  right sensor; 3 = left sensor; recurse=> true = which downto 0
void read_ultra(int which, bool recurse){
    if(which == 0){
        return;
    }
    if(which == 1){
		    cm[0] = sonar[0].ping_cm(30) ;
        Serial.print("Distance left: ");
        Serial.print(cm[0]);
        Serial.println(" ");

    }
    if(which == 2){
		    cm[1] = sonar[1].ping_cm(30) ;
        Serial.print("Distance front: "); 
        Serial.print(cm[1]);  
        Serial.println(" ");
    }
    if(which == 3){
        cm[2] = sonar[2].ping_cm(30) ;
        Serial.print("Distance right: ");
        Serial.print(cm[2]);
        Serial.println(" ");
    }
    if(recurse){
      delay(100);
        which--;
        read_ultra(which,recurse);
    }
}
void checkWalls(){ //make the front one different. could be error.
	 for(int i=0;i < sizeof(cm);i++){
		 if(cm[0]<10){
			 walls[i]=true;
		 }
	 }
}
//////////////////////////////COLOR SENSOR FUNCTIONS/////////////////////////////////////////

void APDS_setup(){
  if(!apds.begin()){
    Serial.println("failed to initialize device! Please check your wiring.");
  }
  else Serial.println("APDS initialized!");
  //enable color sensign mode
  apds.enableColor(true);
}
//display rgb values
void APDS_GetColors(){
  // Wire.beginTransmission(ADPS_I2C);  // read from device
   //wait for color data to be ready
  while(!apds.colorDataReady()){
    delay(5);
  }
  //get the data and print the different channels
  apds.getColorData(&r, &g, &b, &c);
  Serial.print("red: ");
  Serial.print(r);
  
  Serial.print(" green: ");
  Serial.print(g);
  
  Serial.print(" blue: ");
  Serial.print(b);
  // Wire.endTransmission();      // Stop transmitting
  delay(100);
  
}
//determine color from rgb values and see if blue center is reached
void CheckifSolved(){
    if(b > r && b > g){
       Serial.println(" - BLUE detected!");
       middle=true;
    }
    if( g < 12 && r < 12 && b < 12){  //adjust based on lighting under the robot
      Serial.println(" - BLACK detected!");
    }
}
////////////////////////////////////MPU FUNCTIONS////////////////////////////////////

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
  return X_degrees;
}
//return y degrees


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
