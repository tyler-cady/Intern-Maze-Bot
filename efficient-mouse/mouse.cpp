#include "mouse.h"

Mouse::Mouse() {}

void Mouse::setup() {
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
    APDS_setup();
}

void Mouse::updateEncoder() {
    int MSB = digitalRead(encoderPin1);
    int LSB = digitalRead(encoderPin2);
    int encoded = (MSB << 1) | LSB;
    int sum  = (lastEncoded << 2) | encoded;

    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue++;
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue--;

    lastEncoded = encoded;
}

void Mouse::updateEncoder2() {
    int MSB = digitalRead(encoder2Pin1);
    int LSB = digitalRead(encoder2Pin2);
    int encoded = (MSB << 1) | LSB;
    int sum  = (lastEncoded2 << 2) | encoded;

    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue2--;
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue2++;

    lastEncoded2 = encoded;
}

void Mouse::updateEncoders() { /* Trying hard ... */
    // Read encoder 1
    int MSB1 = digitalRead(encoderPin1);
    int LSB1 = digitalRead(encoderPin2);
    int encoded1 = (MSB1 << 1) | LSB1;
    int sum1  = (lastEncoded << 2) | encoded1;

    if(sum1 == 0b1101 || sum1 == 0b0100 || sum1 == 0b0010 || sum1 == 0b1011) encoderValue++;
    if(sum1 == 0b1110 || sum1 == 0b0111 || sum1 == 0b0001 || sum1 == 0b1000) encoderValue--;

    lastEncoded = encoded1;

    // Read encoder 2
    int MSB2 = digitalRead(encoder2Pin1);
    int LSB2 = digitalRead(encoder2Pin2);
    int encoded2 = (MSB2 << 1) | LSB2;
    int sum2  = (lastEncoded2 << 2) | encoded2;

    if(sum2 == 0b1101 || sum2 == 0b0100 || sum2 == 0b0010 || sum2 == 0b1011) encoderValue2--;
    if(sum2 == 0b1110 || sum2 == 0b0111 || sum2 == 0b0001 || sum2 == 0b1000) encoderValue2++;

    lastEncoded2 = encoded2;

    if (isTurn == false) { // Straight motion
        Input = (encoderValue - encoderValue2) / 2.0;  // Average encoder difference
        Setpoint = 0;  // Target is equal speed for both motors
        myPID.Compute();
        setSpeed(Output, PWMright1);
        setSpeed(Output, PWMright2);
        setSpeed(Output, PWMleft1);
        setSpeed(Output, PWMleft2);
    } else if (isTurn == true) { // Turning (target correct relative speeds)
        Input = (encoderValue + encoderValue2) / 2.0;  // Average encoder sum for turning
        Setpoint = degree * (PPR / 360.0); // Convert degree to encoder count
        myPID.Compute();
        if (R_L) { // Right turn
            setSpeed(Output, PWMright1);
            setSpeed(0, PWMright2);
            setSpeed(0, PWMleft1);
            setSpeed(Output, PWMleft2);
        } else { // Left turn
            setSpeed(0, PWMright1);
            setSpeed(Output, PWMright2);
            setSpeed(Output, PWMleft1);
            setSpeed(0, PWMleft2);
        }
    }
}

int Mouse::getRotations(bool R_L) const { return static_cast<int>((R_L ? encoderValue1 : encoderValue2) / PPR);}

void Mouse::resetEncoders() {
    encoderValue = 0;
    encoderValue2 = 0;
}

void Mouse::readEncoders() {
    Serial.print("Encoder1: ");
    Serial.print(encoderValue, DEC);
    Serial.println(" ");
    Serial.print("Encoder2: ");
    Serial.print(encoderValue2, DEC);
    Serial.println(" "); 
}

void Mouse::setSpeed(float speed, int pin) {
    float percent = float(speed / 100);
    float dutycycle = float(255 * percent);
    analogWrite(pin, dutycycle);
}

void Mouse::turn(bool R_L, int degree, int turn_speed) {
    isTurn = true;
    if (R_L) {
        setSpeed(turn_speed, PWMright1);
        setSpeed(0, PWMright2);
        setSpeed(turn_speed, PWMleft1);
        setSpeed(0, PWMleft2);
    } else {
        setSpeed(turn_speed, PWMright1);
        setSpeed(0, PWMright2);
        setSpeed(turn_speed, PWMleft1);
        setSpeed(0, PWMleft2);
    }
    motors_stop(2);
    resetEncoders();
}

void Mouse::motors_stop(int R_L_BOTH) {
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

void Mouse::motors_straight(bool direction, int speed) {
    isTurn = false;
    if (direction) {
        setSpeed(0, PWMright1);
        setSpeed(speed, PWMright2);
        setSpeed(speed, PWMleft1);
        setSpeed(0, PWMleft2);
    } else {
        setSpeed(speed, PWMright1);
        setSpeed(0, PWMright2);
        setSpeed(0, PWMleft1);
        setSpeed(speed, PWMleft2);
    }
}

float Mouse::read_ultra(int which) {
    float distance = -1; // Default to -1 if invalid which parameter
    if (which >= 1 && which <= 3) {
        distance = sonar[which - 1].ping_cm(30);
        cm[which - 1] = distance;
    }
    return distance;
}

void Mouse::checkWalls() {
    for(int i = 0; i < 3; i++) {
        cm[i] = read_ultra(i);
        if(cm[i] < 10) {
            walls[i] = true;
        }
    }
}

void Mouse::APDS_setup() { // Move to setup loop? 
    if (!apds.begin()) { 
        Serial.println("failed to initialize device! Please check your wiring.");
    } else {
        Serial.println("Device initialized!");
    }
    apds.enableColor(true);
}

void Mouse::CheckifSolved() {
    apds.getColorData(&r, &g, &b, &c);
    if ((r > 100) && (g > 100) && (b > 100) && (c > 500)) {
        motors_stop(2);
    }
}
void Mouse::MPU_calibrate() {
    int numReadings = 100;
    float xSum[4] = {0, 0, 0, 0};
    float ySum[4] = {0, 0, 0, 0};
    float zSum[4] = {0, 0, 0, 0};
    int orientationIndex = 0;
    
    Serial.println("Starting MPU calibration. Please hold the bot steady in each orientation as prompted.");

    const char* orientations[] = {
        "forward",
        "90 degrees right",
        "90 degrees left",
        "backwards"
    };

    for (int orientation = 0; orientation < 4; orientation++) {
        Serial.print("Place the bot ");
        Serial.print(orientations[orientation]);
        Serial.println(" White on color sensor to continue.");
        while (!Serial.available()) {
            apds.getColorData(&r, &g, &b, &c);
            if ((r > 200) && (g > 200) && (b > 200) && (c > 500)) {
                
            }
        }
        
        Serial.read(); // Clear the input buffer

        // Collect multiple readings for better accuracy
        for (int i = 0; i < numReadings; i++) {
            if (!dmpReady) return;

            // Check if new FIFO packet is ready
            if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetEuler(euler, &q);

                xSum[orientation] += euler[0];
                ySum[orientation] += euler[1];
                zSum[orientation] += euler[2];
            }
            delay(20); // Delay to allow for stable readings
        }
    }

    // Calculate the average offsets
    xOffset = (xSum[0] + xSum[1] + xSum[2] + xSum[3]) / (4 * numReadings);
    yOffset = (ySum[0] + ySum[1] + ySum[2] + ySum[3]) / (4 * numReadings);
    zOffset = (zSum[0] + zSum[1] + zSum[2] + zSum[3]) / (4 * numReadings);

    Serial.println("Calibration complete.");
    Serial.print("xOffset: "); Serial.println(xOffset);
    Serial.print("yOffset: "); Serial.println(yOffset);
    Serial.print("zOffset: "); Serial.println(zOffset);
}

void Mouse::MPU_getdata() {
    if (!dmpReady) return;
    while (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);
        Serial.print("X: ");
        Serial.print(MPU_getX());
        Serial.print("\tY: ");
        Serial.print(MPU_getY());
        Serial.print("\tZ: ");
        Serial.print(MPU_getZ());
        Serial.println();
    }
}

float Mouse::MPU_getX() const { return (euler[0] * 180/M_PI) - xOffset; }

float Mouse::MPU_getY() const { return (euler[1] * 180/M_PI) - yOffset; }

float Mouse::MPU_getZ() { return (euler[2] * 180/M_PI) - zOffset; }

void Mouse::MPU_setup() {
    Wire.begin();
    Wire.setClock(400000);
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}
