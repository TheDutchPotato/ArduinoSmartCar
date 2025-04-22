#include "Arduino.h"
//File for definition of device function and setting variables
#include "DeviceDriverSet_x.h"
#include <avr/wdt.h>

void DeviceDriverSet_Motor::DeviceDriverSet_Motor_Init(void) //Set pins
{
  pinMode(PIN_Motor_PWM_L, OUTPUT);
  pinMode(PIN_Motor_PWM_R, OUTPUT);
  pinMode(PIN_Motor_LIN_1, OUTPUT);
  pinMode(PIN_Motor_RIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
};

void DeviceDriverSet_Motor::DeviceDriverSet_Motor_Control(
    boolean group_L, uint8_t speed_L,
    boolean group_R, uint8_t speed_R)
{
    // Enable motor driver
    digitalWrite(PIN_Motor_STBY, HIGH);

    // Control left motor group
    if (group_L) {
        digitalWrite(PIN_Motor_LIN_1, LOW);
    } else {
        digitalWrite(PIN_Motor_LIN_1, HIGH);
    }
    analogWrite(PIN_Motor_PWM_L, speed_L);

    // Control right motor group
    if (group_R) {
        digitalWrite(PIN_Motor_RIN_1, LOW);
    } else {
        digitalWrite(PIN_Motor_RIN_1, HIGH);
    }
    analogWrite(PIN_Motor_PWM_R, speed_R);
}

void DeviceDriverSet_Ultrasonic::DeviceDriverSet_Ultrasonic_Init(void)
{
  pinMode(PIN_Echo, INPUT);
  pinMode(PIN_Trigger, OUTPUT);
};

void DeviceDriverSet_LineTrack::DeviceDriverSet_Linetrack_Init(void)
{
  pinMode(PIN_LineTrack_L, INPUT);
  pinMode(PIN_LineTrack_M, INPUT);
  pinMode(PIN_LineTrack_R, INPUT);

  // Run calibration during initialization
  calibrateSensors();
}

void DeviceDriverSet_LineTrack::calibrateSensors()
{
  Serial.println("Calibrating line tracking sensors...");
  for (int i = 0; i < 100; i++) { // Take 100 readings for calibration
    int leftSensor = analogRead(PIN_LineTrack_L);
    int middleSensor = analogRead(PIN_LineTrack_M);
    int rightSensor = analogRead(PIN_LineTrack_R);

    // Update min and max values for each sensor
    if (leftSensor < leftSensorMin) leftSensorMin = leftSensor;
    if (leftSensor > leftSensorMax) leftSensorMax = leftSensor;

    if (middleSensor < middleSensorMin) middleSensorMin = middleSensor;
    if (middleSensor > middleSensorMax) middleSensorMax = middleSensor;

    if (rightSensor < rightSensorMin) rightSensorMin = rightSensor;
    if (rightSensor > rightSensorMax) rightSensorMax = rightSensor;

    delay(10); // Small delay between readings
  }

  // Add a buffer to the max values to account for black line detection
  leftSensorMax += 50;
  middleSensorMax += 50;
  rightSensorMax += 50;

  Serial.println("Calibration complete.");
  Serial.print("Left Sensor Min: "); Serial.print(leftSensorMin); Serial.print(", Max: "); Serial.println(leftSensorMax);
  Serial.print("Middle Sensor Min: "); Serial.print(middleSensorMin); Serial.print(", Max: "); Serial.println(middleSensorMax);
  Serial.print("Right Sensor Min: "); Serial.print(rightSensorMin); Serial.print(", Max: "); Serial.println(rightSensorMax);
}

bool DeviceDriverSet_LineTrack::isOnLine(int sensorValue, int sensorMin, int sensorMax) {
  // Returns true if the sensor is detecting a line (dark surface)
  return sensorValue > ((sensorMax + sensorMin) / 2);
}

int DeviceDriverSet_LineTrack::getLinePosition() {
  // Read all sensors
  int leftValue = analogRead(PIN_LineTrack_L);
  int middleValue = analogRead(PIN_LineTrack_M);
  int rightValue = analogRead(PIN_LineTrack_R);
  
  // Check if each sensor is on the line
  bool leftOnLine = isOnLine(leftValue, leftSensorMin, leftSensorMax);
  bool middleOnLine = isOnLine(middleValue, middleSensorMin, middleSensorMax);
  bool rightOnLine = isOnLine(rightValue, rightSensorMin, rightSensorMax);
  
  // Determine position of line
  if (middleOnLine) {
    return 0;  // Line is in the center
  } else if (leftOnLine) {
    return -1; // Line is to the left
  } else if (rightOnLine) {
    return 1;  // Line is to the right
  }
  
  return 2; // Line is lost
}

