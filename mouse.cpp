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

void Mouse::loop() {
    APDS_GetColors();
    delay(100);
    CheckifSolved();
    delay(100);
    read_ultra(3, true);
    delay(100);
    MPU_getdata();
    readEncoders();
    int rotations_right = getRotations(true);
    int rotations_left = getRotations(false);
    delay(50);
    resetEncoders();
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

int Mouse::getRotations(bool R_L) {
    if (R_L) {
        return int(encoderValue / PPR);
    } else {
        return int(encoderValue2 / PPR);
    }
}

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

void Mouse::Turn(bool R_L, int degree, int turn_speed) {
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

void Mouse::read_ultra(int which, bool recurse) {
    if (which == 0) {
        return;
    }
    if (which == 1) {
        cm[0] = sonar[0].ping_cm(30);
        Serial.print("Distance left: ");
        Serial.print(cm[0], DEC);
        Serial.println(" ");
    }
    if (which == 2) {
        cm[1] = sonar[1].ping_cm(30);
        Serial.print("Distance front: ");
        Serial.print(cm[1], DEC);
        Serial.println(" ");
    }
    if (which == 3) {
        cm[2] = sonar[2].ping_cm(30);
        Serial.print("Distance right: ");
        Serial.print(cm[2], DEC);
        Serial.println(" ");
    }
    if (recurse) {
        delay(100);
        which--;
        read_ultra(which, recurse);
    }
}

void Mouse::checkWalls() {
    for(int i = 0; i < 3; i++) {
        if(cm[i] < 10) {
            walls[i] = true;
        }
    }
}

void Mouse::APDS_setup() {
    if (!apds.begin()) { 
        Serial.println("failed to initialize device! Please check 
 your wiring.");
    } else {
        Serial.println("Device initialized!");
    }
    apds.enableColor(true);
}

void Mouse::APDS_GetColors() {
    apds.getColorData(&r, &g, &b, &c);
    Serial.print("red: ");
    Serial.print(r);
    Serial.print(" green: ");
    Serial.print(g);
    Serial.print(" blue: ");
    Serial.print(b);
    Serial.print(" clear: ");
    Serial.println(c);
}

void Mouse::CheckifSolved() {
    if ((r > 100) && (g > 100) && (b > 100) && (c > 500)) {
        motors_stop(2);
    }
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

float Mouse::MPU_getX() {
    return euler[0] * 180/M_PI;
}

float Mouse::MPU_getY() {
    return euler[1] * 180/M_PI;
}

float Mouse::MPU_getZ() {
    return euler[2] * 180/M_PI;
}

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
