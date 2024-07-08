#include "EncoderRPM.h"

volatile long EncoderRPM::_pulseCount = 0;

EncoderRPM::EncoderRPM(int pinA, int pinB) {
  _pinA = pinA;
  _pinB = pinB;
  _lastPulseCount = 0;
  _lastTime = 0;
}

void EncoderRPM::begin() {
  pinMode(_pinA, INPUT);
  pinMode(_pinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(_pinA), _isr, CHANGE);
}

void EncoderRPM::update() {
  unsigned long currentTime = millis();
  if (currentTime - _lastTime >= 1000) {
    _lastTime = currentTime;
    _lastPulseCount = _pulseCount;
    _pulseCount = 0; // Reset pulse count after each second
  }
}

float EncoderRPM::getRPM() {
  return (_lastPulseCount / 20.0) * 60.0;
}

void EncoderRPM::_isr() {
  _pulseCount++;
}
