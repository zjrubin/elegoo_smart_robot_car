#pragma once

#include <stdint.h>

class String;

extern String CommandSerialNumber;

/*CMD_MotorControl: Motor Control： Motor Speed、Motor Direction、Motor Time*/
extern uint8_t CMD_MotorSelection;
extern uint8_t CMD_MotorDirection;

extern uint16_t CMD_MotorSpeed;
extern unsigned long CMD_leftMotorControl_Millis;
extern unsigned long CMD_rightMotorControl_Millis;

/*CMD_CarControl: Car Control：Car moving direction、Car Speed、Car moving time*/
extern uint8_t CMD_CarDirection;
extern uint8_t CMD_CarSpeed;
extern uint16_t CMD_CarTimer;
extern unsigned long CMD_CarControl_Millis;

extern uint8_t CMD_CarDirectionxxx;
extern uint8_t CMD_CarSpeedxxx;
extern uint16_t CMD_Distance;

/*
  Bluetooth remote control mode
*/
void bluetooth_mode();

/*
  N21:command
  CMD mode：Ultrasonic module：App controls module status, module sends data to app
*/
void CMD_UltrasoundModuleStatus_Plus(uint8_t is_get);

/*
  N22:command
   CMD mode：Tracking module：App controls module status, module sends data to app
*/
void CMD_TraceModuleStatus_Plus(uint8_t is_get);

/*
  N1:command
  CMD mode：Sport mode <motor control> Control motor by app
  Input：uint8_t is_MotorSelection,  Motor selection   1：left  2：right  0：all
        uint8_t is_MotorDirection,   Motor steering  1：Forward  2：Reverse 0：stop
        uint8_t is_MotorSpeed,       Motor speed   0-250
*/
void CMD_MotorControl_Plus(uint8_t is_MotorSelection, uint8_t is_MotorDirection, uint8_t is_MotorSpeed);

/*
  N4：command
  CMD mode：<Car control> APP control car
  Time limited
*/
void CMD_CarControl_Plus(uint8_t is_CarDirection, uint8_t is_CarSpeed, uint8_t is_Timer);

/*
  N40：command
  CMD mode：<Car control> APP control car
  No time limit
*/
void CMD_CarControl_Plusxxx(uint8_t is_CarDirection, uint8_t is_CarSpeed);

/*
  N5:command
  CMD mode：
*/
void CMD_ClearAllFunctionsXXX(void);

void getDistance_xx(void);

/*
  Bluetooth serial port data acquisition and control command parsing
*/
void getBTData_Plus(void);
