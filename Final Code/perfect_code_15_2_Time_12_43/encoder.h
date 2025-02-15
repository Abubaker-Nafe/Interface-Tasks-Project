#ifndef ENCODER_H
#define ENCODER_H

extern volatile long pulseCount;  

void EncoderSetup();
float EncoderReading();

#endif // ENCODER_H
