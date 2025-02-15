#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "encoder.h"

// ----------------------------
// Pin Definitions
// ----------------------------
// Motor direction pins
#define IN1 13   // Right motor direction pin 1
#define IN2 12   // Right motor direction pin 2
#define IN3 14   // Left motor direction pin 1
#define IN4 27   // Left motor direction pin 2

// Motor speed (PWM) pins
#define ENA 26   // Right motor PWM
#define ENB 25   // Left motor PWM

// Encoder pins
#define ENCA_A 4 
#define ENCA_B 18 // left : 222
#define ENCB_A 19 // right : 203
#define ENCB_B 23

// IR sensor pins
#define IR_LEFT 34
#define IR_RIGHT 35

// for distance
#define WHEEL_DIAMETER_CM 4.4        
#define PI 3.14159                   

#define NUM_ROTATIONS 2.7
#define NUM_ROTATIONS_AFTER_TURNING 2.9

#define TARGET_DISTANCE_CM (NUM_ROTATIONS * PI * WHEEL_DIAMETER_CM)
#define TARGET_DISTANCE_CM_AFTER_TURNING (NUM_ROTATIONS_AFTER_TURNING * PI * WHEEL_DIAMETER_CM)

// ----------------------------
// Global Sensor & Control Objects
// ----------------------------
VL53L0X lidar;
Adafruit_MPU6050 mpu;

// ----------------------------
// Global Variables for Encoders and PID (Forward Motion)
// ----------------------------
// Encoder pulse counts (updated inside ISRs)
volatile long pulseCountA = 0;  // Left motor encoder count
volatile long pulseCountB = 0;  // Right motor encoder count

// Desired (feed‑forward) speeds for both motors (0–255)
int desiredSpeedLeft = 60;   // For the left motor (remember: wiring is reversed)
int desiredSpeedRight = 60;  // For the right motor

// PID Control Variables (for balancing the speeds)
unsigned long lastPIDTime = 0;   // Last time (in ms) the PID loop was run
long lastPulseA = 0;             // Encoder A count from last PID update
long lastPulseB = 0;             // Encoder B count from last PID update
float errorSum = 0;              // Integral term (accumulated error)
float lastError = 0;             // Previous error value

// PID gains (tune these values)
const float Kp = 2.1;    // Proportional gain
const float Ki = 0.2;    // Integral gain
const float Kd = 0.0;    // Derivative gain

int minMotorSpeed = 50; // minimum speed to keep the motors running

// Maximum absolute value for the integral term (anti-windup)
const float maxIntegral = 1000.0;

// ----------------------------
// Global Variables for MPU (Turning)
// ----------------------------
unsigned long lastTime = 0;   // For integrating gyro data (in turning mode)
float currentYaw = 0.0;       // Accumulated yaw (in degrees)
bool turningInProgress = false;  // Flag to indicate when a turn is underway

// ----------------------------
// Function Prototypes
// ----------------------------
void IRAM_ATTR encoderAInterrupt();
void IRAM_ATTR encoderBInterrupt();
void IRAM_ATTR encoderCInterrupt();
void IRAM_ATTR encoderDInterrupt();
void updatePID();  // PID balancing for forward motion
void setMotorDirectionForward();
void setMotorDirectionReverse();
void setMotorDirectionTurnLeft();
void setMotorDirectionTurnRight();
void performTurn90(bool turnLeft);  // turnLeft==true means a left turn; false means right
void performTurn180(bool turnLeft);
void stopMotors();
void moveForwardDistance(float distanceCM);

// New helper functions for sensor reading and decision‐making:
bool readIRLeft();
bool readIRRight();
int readLidarDistance();
void processSensorInputs();
volatile bool flag = true;

// ----------------------------
// Setup
// ----------------------------
void setup() {
  Serial.begin(115200);
  
  // Set motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  EncoderSetup();

  // Initialize IR sensor pins as inputs
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  
  // Initialize last PID time
  lastPIDTime = millis();
  
  // I2C Setup: Use explicit SDA (P21) and SCL (P22) pins so both LiDAR and MPU work.
  Wire.begin(21, 22);
  delay(100);
  
  // MPU6050 Setup using the Adafruit library
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }

  // LiDAR Setup
  lidar.init();
  lidar.setTimeout(500);
  
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(1000);
  lastTime = millis();
  currentYaw = 0.0;
}

// ----------------------------
// Main Loop
// ----------------------------
void loop() {

  updatePID();
  processSensorInputs();
  
  delay(10);  // Short delay for loop stability
}

// ----------------------------
// Sensor Reading Helper Functions
// ----------------------------
bool readIRLeft() {
  return digitalRead(IR_LEFT);
}

bool readIRRight() {
  return digitalRead(IR_RIGHT);
}

int readLidarDistance() {
  // Return distance in cm.
  return lidar.readRangeSingleMillimeters() / 10;
}

// ----------------------------
// Decision-Making Function
// ----------------------------
void processSensorInputs() {
  bool irLeft = readIRLeft();    // HIGH means triggered
  bool irRight = readIRRight();  // HIGH means triggered
  int distance = readLidarDistance();
  
  Serial.print("Distance (cm): ");
  Serial.println(distance);
  
  // The decision tree is based on sensor values.
if(distance <= 9 && irLeft == LOW && irRight == LOW){
    // obstacles everywhere.
    // turn 180 degrees.
    // Then move forward.
    performTurn180(true);
  }else if (irLeft == LOW && irRight == HIGH) {
    Serial.println("Obstacle on left, turning right.");
    // Turn right means the gyro's Z reading should be inverted so that the integration is positive.
    performTurn90(true); // true => turn right
  }else if (irLeft == HIGH && irRight == LOW) {
    Serial.println("Obstacle on right, turning left.");
    performTurn90(false);  // false => turn left
  }else if (distance > 10 && irLeft == LOW && irRight == LOW) {
    // When both IR sensors are triggered, drive forward.
    moveForwardDistance(TARGET_DISTANCE_CM);
  }
  else if (irLeft == HIGH && irRight == HIGH){
    
    changeFlag();
    if(flag){
      performTurn90(false); // false => turn right
    }else{
      performTurn90(true); // true => turn left
    }
  }
  
  delay(10);  // Short delay for loop stability
}

// inverting between the flags (Left & Right)
void changeFlag(){
  flag = !flag;
}

// ----------------------------
// PID Balancing Function (for forward motion)
// ----------------------------
void updatePID() {
  // Skip PID update if a turn is underway.
  if (turningInProgress) return;
  
  unsigned long now = millis();
  unsigned long dt = now - lastPIDTime;
  
  // Update the PID at least every 10 ms.
  if (dt >= 10) {
    float dtSec = dt / 1000.0;
    
    // Calculate the change in encoder counts since the last update.
    long deltaA = pulseCountA - lastPulseA; // Left motor 
    long deltaB = pulseCountB - lastPulseB; // Right motor
    
    // Compute error: here, error is the difference between the speeds.
    // A positive error indicates that the left motor is running slower than the right.
    long error = deltaB - deltaA;
    
    // Update the integral term with anti-windup.
    errorSum += error * dtSec;
    errorSum = constrain(errorSum, -maxIntegral, maxIntegral);
    
    // Compute the derivative term.
    float derivative = (dtSec > 0) ? (error - lastError) / dtSec : 0;
    
    // Compute the PID correction.
    float correction = Kp * error + Ki * errorSum + Kd * derivative;
    
    // Adjust the PWM signals based on the desired speeds plus the PID correction.
    // The correction is applied differentially: subtract from one side and add to the other.
    int leftPWM = desiredSpeedLeft - correction;
    int rightPWM = desiredSpeedRight + correction;
    
    leftPWM = constrain(leftPWM, 0, 255);
    rightPWM = constrain(rightPWM, 0, 255);
    
    // Drive the motors with the calculated PWM values.
    // (Note: Based on your wiring, the left motor's wiring is reversed.)
    analogWrite(ENA, rightPWM); // Right motor PWM output
    analogWrite(ENB, leftPWM);  // Left motor PWM output
    
    // Save the current encoder counts and error for the next cycle.
    lastPulseA = pulseCountA;
    lastPulseB = pulseCountB;
    lastError = error;
    lastPIDTime = now;
  }
}

// ----------------------------
// Turning Function Using MPU (90° Turn)
// ----------------------------
void performTurn90(bool turnLeft) {
  turningInProgress = true; // Disable PID updates during the turn.
  
  // Command the motors to turn.
  if (turnLeft) {
    setMotorDirectionTurnLeft();
  } else {
    setMotorDirectionTurnRight();
  }
  
  // Reset the yaw integration.
  currentYaw = 0.0;
  lastTime = millis();
  
  // Continue integrating gyro data until the robot has turned ~90°.
  while (true) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;
    
    // Get the gyro Z-axis reading in degrees per second.
    float gyroZDegreesPerSec = g.gyro.z * (180.0 / PI);
    // For a right turn, invert the sign so that the integrated angle increases positively.
    if (!turnLeft) {
      gyroZDegreesPerSec = -gyroZDegreesPerSec;
    }
    
    // Integrate the gyro reading.
    currentYaw += gyroZDegreesPerSec * dt;
    float angleTurned = abs(currentYaw);
    
    if (angleTurned >= 80.0) {  // slightly less than 90° to allow for overshoot
      break;
    }
    delay(10);
  }
  
  // Stop the motors briefly to settle the turn.
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delay(200);
  
  // Resume forward motion.
  moveForwardDistance(TARGET_DISTANCE_CM_AFTER_TURNING);
  
  turningInProgress = false;
}

// ----------------------------
// Turning Function (180° Turn)
// ----------------------------
void performTurn180(bool turnLeft) {
  turningInProgress = true; // Disable PID updates during the turn.
  
  // Command the motors to turn.
  if (turnLeft) {
    setMotorDirectionTurnLeft();
  } else {
    setMotorDirectionTurnRight();
  }
  
  // Reset the yaw integration.
  currentYaw = 0.0;
  lastTime = millis();
  
  // Continue integrating gyro data until the robot has turned ~180°.
  while (true) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;
    
    float gyroZDegreesPerSec = g.gyro.z * (180.0 / PI);
    if (!turnLeft) {
      gyroZDegreesPerSec = -gyroZDegreesPerSec;
    }
    
    currentYaw += gyroZDegreesPerSec * dt;
    float angleTurned = abs(currentYaw);
    
    Serial.print("Turning: angle turned = ");
    Serial.print(angleTurned);
    Serial.println("°");
    
    if (angleTurned >= 170.0) {  // stop just before 180° to account for inertia
      break;
    }
    delay(10);
  }
  
  // Stop the motors briefly.
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delay(200);
  
  // Resume forward motion.
  moveForwardDistance(TARGET_DISTANCE_CM_AFTER_TURNING);
  
  turningInProgress = false;
}

// ----------------------------
// Motor Direction Helper Functions
// ----------------------------
void setMotorDirectionForward() {
  // For forward motion:
  digitalWrite(IN1, HIGH);  // Right motor forward
  digitalWrite(IN2, LOW);
  // For the left motor (wiring reversed):
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void setMotorDirectionReverse() {
  // For reverse motion:
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void setMotorDirectionTurnLeft() {
  // For turning left:
  digitalWrite(IN1, LOW);   // Right motor forward
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);   // Left motor reversed
  digitalWrite(IN4, HIGH);
}

void setMotorDirectionTurnRight() {
  // For turning right:
  digitalWrite(IN1, HIGH);  // Right motor reversed
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// ----------------------------
// Move Forward a Given Distance Using Encoder Feedback
// ----------------------------
// Define new PID constants for distance and heading (tune as needed)
const float dKp = 2.1;    // Distance proportional gain
const float dKi = 0.2;    // Distance integral gain
const float dKd = 0.0;    // Distance derivative gain

const float hKp = 2.1;    // Heading proportional gain
const float hKi = 0.2;    // Heading integral gain
const float hKd = 0.0;    // Heading derivative gain

// A maximum integral value to prevent windup (can be adjusted)

// Ensure you have a variable to track the heading:
float desiredHeading = 0.0;     // Desired heading (e.g., 0 for straight)

// Make sure to reset these as needed before starting movement.
void moveForwardDistance(float distanceCM) {
  // Reset encoder count
  noInterrupts();
  pulseCount = 0;   // Adjust based on your encoder variable name
  interrupts();

  // Reset heading if desired (or use your current heading as the baseline)
  currentYaw = 0.0;  
  desiredHeading = 0.0; // For example, drive straight ahead

  // Set motors for forward movement
  setMotorDirectionForward();

  // Variables for distance PID
  unsigned long prevTime = millis();
  float prevDistanceError = distanceCM;
  float distanceIntegral = 0.0;

  // Variables for heading PID
  float headingErrorSum = 0.0;
  float lastHeadingError = 0.0;

  while (EncoderReading() < distanceCM) {
    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0;
    if (dt <= 0) dt = 0.001;  // Avoid division by zero
    prevTime = currentTime;

    // *** Distance PID ***
    float currentDistance = EncoderReading();
    float distanceError = distanceCM - currentDistance;
    distanceIntegral += distanceError * dt;
    float distanceDerivative = (distanceError - prevDistanceError) / dt;
    prevDistanceError = distanceError;
    float distancePIDOutput = dKp * distanceError + dKi * distanceIntegral + dKd * distanceDerivative;
    // Constrain the base speed
    int baseSpeed = (int)constrain(distancePIDOutput, minMotorSpeed, 255);

    // *** Heading PID ***
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    // Convert gyro z-axis reading to degrees per second
    float gyroZ = g.gyro.z * (180.0 / PI);
    // Integrate to update the current heading
    currentYaw += gyroZ * dt;
    // Calculate heading error (difference between desired and current heading)
    float headingError = desiredHeading - currentYaw;
    headingErrorSum += headingError * dt;
    headingErrorSum = constrain(headingErrorSum, -maxIntegral, maxIntegral);
    float headingDerivative = (dt > 0) ? (headingError - lastHeadingError) / dt : 0;
    lastHeadingError = headingError;
    float headingCorrection = hKp * headingError + hKi * headingErrorSum + hKd * headingDerivative;

    // *** Combine the two PIDs ***
    // Differential drive: add the heading correction to one side and subtract from the other.
    int leftPWM = constrain(baseSpeed + headingCorrection, 0, 255);
    int rightPWM = constrain(baseSpeed - headingCorrection, 0, 255);

    // Drive motors with the computed PWM values
    analogWrite(ENA, rightPWM);  // Right motor
    analogWrite(ENB, leftPWM);   // Left motor

    // Break the loop if we are close enough to the target distance
    if (distanceError <= 0.5) {
      break;
    }
    delay(10); // Loop delay for stability
  }

  stopMotors();
  delay(500);
}

// ----------------------------
// Stop the Motors
// ----------------------------
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
