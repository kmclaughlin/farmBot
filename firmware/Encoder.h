#ifndef ENCODER_H
#define ENCODER_H

#include "Arduino.h"

class Encoder{
  public:
    Encoder(uint8_t _aByte, uint8_t _bByte, volatile uint8_t *_aPort, 
            volatile uint8_t *_bPort, bool invert);
    void process();
    long getPosition() {return position * inverted;};
    void setPosition(long newPosition) { position = newPosition; };
    int  movementCheck() { return abs(prvMovementCheckPosition - position);};
    void resetMovementCheck() { prvMovementCheckPosition = position;};
  private:
    void readChannels();
    void shiftChannels();
    // pin settings
  
    // channels
    bool prvValChannelA;
    bool prvValChannelB;
    bool curValChannelA;
    bool curValChannelB;

    uint8_t aByte;
    uint8_t bByte;
    volatile uint8_t *aPort;
    volatile uint8_t *bPort;
    
    volatile long position;
    int inverted;
    int prvMovementCheckPosition;
};

#endif

