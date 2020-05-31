#pragma once

#include <stdint.h>

// Forward declarations
class Servo;

#define PIN_Servo 3
extern Servo servo; //  Create a DC motor drive object

/*
 Servo Control angle Setting
*/
void ServoControl(uint8_t angleSetting);

void delays(unsigned long t);
