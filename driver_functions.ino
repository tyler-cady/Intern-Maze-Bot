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
  NewPing(TRIGGER_PIN, ECHO_FRONT,200);
  NewPing(TRIGGER_PIN, ECHO_RIGHT,200);
  NewPing(TRIGGER_PIN, ECHO_LEFT,200);
}

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

// PCICR | = B00000101;
// PCMSK | = B00000001;
// PCMSK2 | = B00000100;
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
int  Red=0, Blue=0, Green=0; 
bool middle = false;
int possible_turns[4]={1,1,1,1}; //Array to keep track of possible turns. {front, right, left, rear}. 1 = turn possble. 0 = no turn possible


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

  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(0, updateEncoder, CHANGE); 
  attachInterrupt(1, updateEncoder, CHANGE);

  attachPCINT(digitalPintoPCINT(ecoder2Pin1),updateEncoder2, CHANGE);
  attachPCINT(digitalPintoPCINT(ecoder2Pin2),updateEncoder2, CHANGE);

  MPU_setup();

}

void loop() {
 setSpeed(0,PWMright1);
 setSpeed(100,PWMright2);


for (int i = 0; i <= 500; i++){
 Serial.print("Forward  ");
 Serial.println(encoderValue);
}

delay(1000);

 setSpeed(100,PWMright1);
 setSpeed(0,PWMright2);

for (int i = 0; i <= 500; i++){
 Serial.print("Reverse  ");
 Serial.println(encoderValue);
}

  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetEuler(euler, &q);
  Serial.print("euler\t");
  Serial.print(euler[0] * 180/M_PI);
  Serial.print("\t");
  Serial.print(euler[1] * 180/M_PI);
  Serial.print("\t");
  Serial.println(euler[2] * 180/M_PI);

delay(1000);

} 



///////////////////////////////////////////////ENCODER FUNCTIONS////////////////////////////////////////////////////
void updateEncoder(){
  int MSB = digitalRead(ecoder2Pin1); //MSB = most significant bit
  int LSB = digitalRead(ecoder2Pin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue ++;

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

/////////////////////////////////////////////MOTORS////////////////////////////////////////////////////
//speed in percentage
void setSpeed(float speed, int pin){
  // analogWriteResolution(8);
  float percent = float(speed/100);
  float dutycycle = float(255*percent);
  analogWrite(pin, dutycycle); //or digitalwrite????
}

// R_L determines turn right or left. R_L = true : right turn; R_L = false : left turn;
void Turn(bool R_L, int degree, int turn_speed) 
{
    // while(degree!=90){
        if (R_L){
            setSpeed(turn_speed,PWMright1);
            setSpeed(0,PWMright2);
            // setSpeed(turn_speed,PWMleft1);
            // setSpeed(0,PWMleft2);
        }
        else{
            setSpeed(turn_speed,PWMright1);
            setSpeed(0,PWMright2);
            // setSpeed(turn_speed,PWMleft1);
            // setSpeed(0,PWMleft2);
        }
    // }
}
//R_L_BOTH: 0=right motor only; 1=left motor only; 2=both motors
void motors_stop(int R_L_BOTH) 
{
    if (R_L_BOTH == 0){
        setSpeed(0,PWMright1);
        setSpeed(0,PWMright2);
    }
    else if (R_L_BOTH == 1){
        // setSpeed(0,PWMleft1);
        // setSpeed(0,PWMleft2);
    }
    else if (R_L_BOTH == 2){
        setSpeed(0,PWMright1);
        setSpeed(0,PWMright2);
        // setSpeed(0,PWMleft1);
        // setSpeed(0,PWMleft2);
    }
}
//direction: true=forwad; false=backwards
void motors_straight(bool direction, int speed) 
{
    if (direction){
        setSpeed(0,PWMright1);
        setSpeed(speed,PWMright2);
        // setSpeed(speed,PWMleft1);
        // setSpeed(0,PWMleft2);
    }
    else{
        setSpeed(speed,PWMright1);
        setSpeed(0,PWMright2);
        // setSpeed(0,PWMleft1);
        // setSpeed(speed,PWMleft2);
    }
}


///////////////////////////////////////////////////////////Ultrasonic/////////////////////////////////////////////////

void updatePossibleTurns()
{
    if (sonar[0].ping_cm() < 2){
        Serial.println("Wall front");
        possible_turns[0]=0;
    }
    else{
        Serial.println("No wall front");
    }
    if (sonar[1].ping_cm()  < 2){
        Serial.println("Wall Right");
        possible_turns[1]=0;
    }
    if (sonar[2].ping_cm()  < 2){
        Serial.println("Wall left");
        possible_turns[2]=0;
    }
}

void read_ultra(int which, bool recurse){
    double cm;
    
    if(which == 0){
        return;
    }
    if(which == 1){
        cm = sonar[0].ping_cm() ;
        Serial.printf("Distance front: %f",cm);   
        Serial.println(" ");
    }
    if(which == 2){
        cm = sonar[1].ping_cm() ;
        Serial.printf("Distance right: %f",cm); 
        Serial.println(" ");
    }
    if(which == 3){
        cm = sonar[2].ping_cm() ;
        Serial.printf("Distance left: %f",cm);
        Serial.println(" ");
    }
    if(recurse){
        which--;
        read_ultra(which,recurse);
    }
}
//////////////////////////////COLOR SENSOR FUNCTIONS/////////////////////////////////////////

void GetColors()  
{    
  digitalWrite(s2,  LOW);                                           //S2/S3 levels define which set  of photodiodes we are using LOW/LOW is for RED LOW/HIGH is for Blue and HIGH/HIGH  is for green 
  digitalWrite(s3, LOW); 
  delay(100);
  Red = pulseIn(out, LOW);       //here we wait  until "out" go LOW, we start measuring the duration and stops when "out" is  HIGH again, if you have trouble with this expression check the bottom of the code
  int RedMap=map(Red,27,172,255,0);
  delay(100);  
  digitalWrite(s2,  LOW);
  digitalWrite(s3, HIGH);                                         //Here  we select the other color (set of photodiodes) and measure the other colors value  using the same techinque
  Blue = pulseIn(out, LOW);
  int BlueMap=map(Blue,27,172,255,0);
  delay(100);
  digitalWrite(s2,  HIGH);
  digitalWrite(s3, HIGH);  
  delay(100);
  Green = pulseIn(out,  LOW);
  delay(100);
  int GreenMap=map(Green,27,172,255,0);
  Serial.printf("RGB: %d %d %d",RedMap,GreenMap,BlueMap);     //output RGB
  Serial.println("");
}


//sets middle = true if robot has reaced the middle.
void CheckifSolved(){
    if (Red > Green && Red > Blue ) {       
      Serial.println("- RED detected!");      
      middle=false;
    }
    if(Blue > Red && Blue > Green){
        Serial.println(" - BLUE detected!");
        motors_stop(2);
        middle=true;
    }
    else{
        motors_straight(true, 100);
    }
    if(Green > Red && Green > Blue){
        Serial.println(" - Green detected!");
    }
}


////////////////////////////////////MPU FUNCTIONS////////////////////////////////////
void MPU_setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(25);
    mpu.setYGyroOffset(-47);
    mpu.setZGyroOffset(-18);
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
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
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

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
