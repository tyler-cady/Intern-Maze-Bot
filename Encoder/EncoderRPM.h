#ifndef EncoderRPM_h
#define EncoderRPM_h

#include "Particle.h"

class EncoderRPM {
  public:
    EncoderRPM(int pinA, int pinB);
    void begin();
    void update();
    float getRPM();
    
  private:
    int _pinA;
    int _pinB;
    volatile long _pulseCount;
    long _lastPulseCount;
    unsigned long _lastTime;
    static void _isr();
};

#endif
