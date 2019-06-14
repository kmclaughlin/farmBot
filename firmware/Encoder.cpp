#include "Encoder.h"

Encoder::Encoder(uint8_t _aByte, uint8_t _bByte, volatile uint8_t *_aPort, 
                  volatile uint8_t *_bPort, bool invert) {
  position = 0;

  curValChannelA = false;
  curValChannelB = false;
  prvValChannelA = false;
  prvValChannelB = false;

  aByte = _aByte;
  bByte = _bByte;
  aPort = _aPort;
  bPort = _bPort;

  readChannels();
  shiftChannels();
  
  if(invert) 
    inverted = -1;
  else
    inverted = 1;

  resetMovementCheck();
}

/* Check the encoder channels for movement according to this specification
                    ________            ________
Channel A          /        \          /        \
             _____/          \________/          \________
                         ________            ________
Channel B               /        \          /        \
             __________/          \________/          \____
                                   __
Channel I                         /  \
             ____________________/    \___________________

rotation ----------------------------------------------------->

*/
void Encoder::process() {
    shiftChannels();
    readChannels();

    // Detect edges on the A channel when the B channel is high
    if (curValChannelB && !prvValChannelA && curValChannelA)
      position--;
    if (curValChannelB && prvValChannelA && !curValChannelA)
      position++;
}

void Encoder::readChannels() {
  //using direct port manipulation instead of digital read as it is much faster
  curValChannelA = *aPort & aByte;
  curValChannelB = *bPort & bByte;
}

void Encoder::shiftChannels(){

  // Save the current enoder status to later on compare with new values

  prvValChannelA = curValChannelA;
  prvValChannelB = curValChannelB;
}

