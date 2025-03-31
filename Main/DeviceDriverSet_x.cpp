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
};

void DeviceDriverSet_Servo::DeviceDriverSet_Servo_Init(/*unsigned int Position_Angle*/void)
{

};