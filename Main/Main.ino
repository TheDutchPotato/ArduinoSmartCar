#include "DeviceDriverSet_x.h"

// Create instances of device driver classes
DeviceDriverSet_Motor motors;
DeviceDriverSet_LineTrack lineTracker;
DeviceDriverSet_Ultrasonic ultrasonic;

// Constants for line following
const int BASE_SPEED = 90;      // Base motor speed (0-255)
const int MAX_ADJUSTMENT = 90;   // Maximum speed adjustment for turning

// PID Constants
const float KP = 35;  // Proportional gain
const float KI = 0.55;  // Integral gain
const float KD = 3.5;  // Derivative gain

// PID Variables
float lastError = 0;
float integral = 0;
unsigned long lastTime = 0;

// Other constants
const int OBSTACLE_DISTANCE = 15; // Distance in cm to stop for obstacles

// Variables for ultrasonic
long duration;
int distance;

void setup() {
  Serial.begin(9600);
  
  // Initialize all devices
  motors.DeviceDriverSet_Motor_Init();
  lineTracker.DeviceDriverSet_Linetrack_Init();
  ultrasonic.DeviceDriverSet_Ultrasonic_Init();
  
  lastTime = millis();

  // Short forward movement after calibration
  Serial.println("Moving forward briefly...");
  motors.DeviceDriverSet_Motor_Control(true, 70, true, 70);  // Move forward at moderate speed
  delay(500);  // Move for 500ms
  motors.DeviceDriverSet_Motor_Control(false, 0, false, 0);  // Stop
  delay(1000);  // Wait 1 second before starting line following
}

int getDistance() {
  // Send ultrasonic pulse
  digitalWrite(PIN_Trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_Trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_Trigger, LOW);
  
  // Read the echo
  duration = pulseIn(PIN_Echo, HIGH);
  
  // Calculate distance in cm
  return duration * 0.034 / 2;
}

float calculatePID(int linePosition) {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  
  // Convert position to error value
  float error;
  switch(linePosition) {
    case FAR_LEFT:
      error = -3.8;
      break;
    case LEFT:
      error = -2;
      break;
    case CENTER:
      error = 0;
      break;
    case RIGHT:
      error = 2;
      break;
    case FAR_RIGHT:
      error = 3.8;
      break;
    default:
      error = 0; 
      break;
  }
  
  // Calculate integral term with anti-windup
  integral = constrain(integral + (error * deltaTime), -MAX_ADJUSTMENT, MAX_ADJUSTMENT);
  
  // Calculate derivative
  float derivative = (error - lastError) / deltaTime;
  lastError = error;
  
  // Calculate PID output
  float output = (KP * error) + (KI * integral) + (KD * derivative);
  
  // Constrain output to maximum adjustment
  return constrain(output, -MAX_ADJUSTMENT, MAX_ADJUSTMENT);
}

void loop() {
  // Check for obstacles first
  distance = getDistance();
  if (distance <= OBSTACLE_DISTANCE) {
    // Stop motors if obstacle detected
    motors.DeviceDriverSet_Motor_Control(false, 0, false, 0);
    delay(100);
    return;
  }

  // Get line position (-1: left, 0: center, 1: right)
  int linePosition = lineTracker.getLinePosition();
  
  // If line is lost, stop motors
  if (linePosition == LINE_LOST) {
    motors.DeviceDriverSet_Motor_Control(false, 0, false, 0);
    integral = 0; // Reset integral term
    return;
  }
  
  // Calculate PID adjustment
  float speedAdjustment = calculatePID(linePosition);
  
  // Calculate motor speeds - note the reversed signs here
  int leftSpeed = BASE_SPEED + speedAdjustment;   
  int rightSpeed = BASE_SPEED - speedAdjustment;  
  // Ensure speeds stay within valid range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Apply motor speeds
  motors.DeviceDriverSet_Motor_Control(true, leftSpeed, true, rightSpeed);
}

