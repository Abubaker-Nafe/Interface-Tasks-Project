#include "encoder.h"

// Constants for the encoder
const int encoderPinB = 19; // Encoder output B 
const int PPR = 222; // Pulses per revolution 
const float wheelDiameter = 4.4; // Diameter of the wheel in cm 
const float wheelCircumference = 3.14159 * wheelDiameter; // Circumference of the wheel
const float distancePerPulse = wheelCircumference / PPR;

// Variable to store pulse count
volatile long pulseCount = 0;

// Interrupt service routine for encoder
void ISR_encoder() {
    pulseCount++;
}

// Setup function for the encoder
void EncoderSetup() {
    // Set encoder pin as input
    pinMode(encoderPinB, INPUT);

    // Attach interrupt to encoder pin
    attachInterrupt(digitalPinToInterrupt(encoderPinB), ISR_encoder, CHANGE);
}
 
// Read the encoder data and calculate distance traveled (in cm)
float EncoderReading() {
    // Detach interrupt to prevent counting pulses during calculation
    detachInterrupt(digitalPinToInterrupt(encoderPinB));

    float distanceTraveled = pulseCount * distancePerPulse;

    // Reattach interrupt for future pulse counting
    attachInterrupt(digitalPinToInterrupt(encoderPinB), ISR_encoder, CHANGE);
    
    return distanceTraveled;
}
