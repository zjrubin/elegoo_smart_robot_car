#pragma once

#include <Arduino.h>

/*Arduino pin is connected to the IR tracking module*/
#define LineTracking_Pin_Right 10
#define LineTracking_Pin_Middle 4
#define LineTracking_Pin_Left 2

#define LineTracking_Read_Right !digitalRead(LineTracking_Pin_Right)   // Right
#define LineTracking_Read_Middle !digitalRead(LineTracking_Pin_Middle) // Middle
#define LineTracking_Read_Left !digitalRead(LineTracking_Pin_Left)     // Left

extern unsigned long LT_PreMillis;

/*
  Line Tracking Mode
*/
void line_tracking_mode(void);
