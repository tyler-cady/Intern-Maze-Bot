#include "mouse.h"

volatile int Mouse::encoderValue1 = 0;
volatile int Mouse::encoderValue2 = 0;
volatile int Mouse::lastEncoded1 = 0;
volatile int Mouse::lastEncoded2 = 0;

Mouse::Mouse() : myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT) {}

void Mouse::setup() {
    for (int pin : PWM_PINS) {
        pinMode(pin, OUTPUT);
    }
    Serial.begin(9600);

    for (int pin : ENCODER_PINS1) {
        pinMode(pin, INPUT_PULLUP);
    }
    for (int pin : ENCODER_PINS2) {
        pinMode(pin, INPUT_PULLUP);
    }

    attachPCINT(digitalPinToPCINT(encoderPin1), updateEncoder1, CHANGE);
    attachPCINT(digitalPinToPCINT(encoderPin2), updateEncoder1, CHANGE);
    attachPCINT(digitalPinToPCINT(encoder2Pin1), updateEncoder2, CHANGE);
    attachPCINT(digitalPinToPCINT(encoder2Pin2), updateEncoder2, CHANGE);

    MPU_setup();
    APDS_setup();
}

void Mouse::updateEncoder(bool isEncoder1) {
    const int *pins = isEncoder1 ? ENCODER_PINS1 : ENCODER_PINS2;
    int &lastEncoded = isEncoder1 ? lastEncoded1 : lastEncoded2;
    int &encoderValue = isEncoder1 ? encoderValue1 : encoderValue2;

    int MSB = digitalRead(pins[0]);
    int LSB = digitalRead(pins[1]);
    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncoded << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue++;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue--;

    lastEncoded = encoded;
}

void Mouse::updateEncoder1() { updateEncoder(true); }
void Mouse::updateEncoder2() { updateEncoder(false); }

void Mouse::updateEncoders() {
    updateEncoder1();
    updateEncoder2();

    Input = (encoderValue1 - encoderValue2) / 2.0;
    Setpoint = isTurn ? degree * (PPR / 360.0) : 0;

    myPID.Compute();
    if (isTurn) {
        if (R_L) {
            setSpeed(Output, PWMright1);
            setSpeed(0, PWMright2);
            setSpeed(0, PWMleft1);
            setSpeed(Output, PWMleft2);
        } else {
            setSpeed(0, PWMright1);
            setSpeed(Output, PWMright2);
            setSpeed(Output, PWMleft1);
            setSpeed(0, PWMleft2);
        }
    } else {
        for (int pin : PWM_PINS) {
            setSpeed(Output, pin);
        }
    }
}

int Mouse::getRotations(bool R_L) const {
    return static_cast<int>((R_L ? encoderValue1 : encoderValue2) / PPR);
}

void Mouse::resetEncoders() {
    encoderValue1 = 0;
    encoderValue2 = 0;
}

void Mouse::readEncoders() const {
    Serial.print("Encoder1: ");
    Serial.println(encoderValue1, DEC);
    Serial.print("Encoder2: ");
    Serial.println(encoderValue2, DEC);
}

void Mouse::setSpeed(float speed, int pin) const {
    analogWrite(pin, static_cast<int>(255 * (speed / 100)));
}

void Mouse::Turn(bool R_L, int degree, int turn_speed) {
    isTurn = true;
    int highPins[] = {PWMright1, PWMleft1};
    int lowPins[] = {PWMright2, PWMleft2};

    for (int i = 0; i < 2; ++i) {
        setSpeed(turn_speed, highPins[i]);
        setSpeed(0, lowPins[i]);
    }
    motors_stop(2);
    resetEncoders();
}

void Mouse::motors_stop(int R_L_BOTH) const {
    int pins[] = {PWMright1, PWMright2, PWMleft1, PWMleft2};
    int stopIndex = (R_L_BOTH == 2) ? 4 : (R_L_BOTH == 0) ? 2 : 4;

    for (int i = (R_L_BOTH == 1) ? 2 : 0; i < stopIndex; ++i) {
        setSpeed(0, pins[i]);
    }
}

void Mouse::motors_straight(bool direction, int speed) {
    isTurn = false;
    int highPins[] = {PWMright2, PWMleft1};
    int lowPins[] = {PWMright1, PWMleft2};

    if (!direction) std::swap(highPins, lowPins);

    for (int i = 0; i < 2; ++i) {
        setSpeed(speed, highPins[i]);
        setSpeed(0, lowPins[i]);
    }
}

float Mouse::read_ultra(int which) {
    if (which < 1 || which > 3) return -1;
    cm[which - 1] = sonar[which - 1].ping_cm(30);
    return cm[which - 1];
}

void Mouse::checkWalls() {
    for (int i = 0; i < 3; ++i) {
        if (cm[i] < 10) {
            walls[i] = true;
        }
    }
}

void Mouse::APDS_setup() {
    if (!apds.begin()) {
        Serial.println("Failed to initialize device! Please check your wiring.");
    } else {
        Serial.println("Device initialized!");
        apds.enableColor(true);
    }
}

void Mouse::CheckifSolved() {
    apds.getColorData(&r, &g, &b, &c);
    if (r > 100 && g > 100 && b > 100 && c > 500) {
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
        Serial.println(MPU_getZ());
    }
}

float Mouse::MPU_getX() const { return euler[0] * 180 / M_PI; }
float Mouse::MPU_getY() const { return euler[1] * 180 / M_PI; }
float Mouse::MPU_getZ() const { return euler[2] * 180 / M_PI; }

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
