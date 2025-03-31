#include <stdint.h>
//Header file for classes definition and defining pinout/variables
#ifndef _DeviceDriverSet_x_H_
#define _DeviceDriverSet_x_H_

#include <Arduino.h>

//Motor
class DeviceDriverSet_Motor
{
public:
  void DeviceDriverSet_Motor_Init(void);

  void DeviceDriverSet_Motor_Control(
    boolean group_L, uint8_t speed_L,
    boolean group_R, uint8_t speed_R
  );

private: /*motor pins*/
  #define PIN_Motor_PWM_L 5 //Left Motor group
  #define PIN_Motor_PWM_R 6 //Right Motor group
  #define PIN_Motor_LIN_1 8
  #define PIN_Motor_RIN_1 7
  #define PIN_Motor_STBY 3
};

//Servo
class DeviceDriverSet_Servo
{
public:
  void DeviceDriverSet_Servo_Init(void);

private:
#define PIN_Servo_z 10  //Pin Z-axis 
#define PIN_Servo_y 11  //Pin Y-axis
};

//Ultrasonic sensor
class DeviceDriverSet_Ultrasonic
{
public:
  void DeviceDriverSet_Ultrasonic_Init(void);
  
private:
#define PIN_Trigger 13  //Trigger pin sensor
#define PIN_Echo 12     //Echo pin sensor
};

//Line tracking sensor
class DeviceDriverSet_LineTrack
{
 public:
   void DeviceDriverSet_Linetrack_Init(void);

private:
  #define PIN_LineTrack_L A2  //Left sensor
  #define PIN_LineTrack_M A1  //Middle sensor
  #define PIN_LineTrack_R A0  //Right sensor
};

#endif