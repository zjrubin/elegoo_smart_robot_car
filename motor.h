#pragma once

#include <WString.h>

/*Arduino pin is connected to the Motor drive module*/
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

#define carSpeed 250 //PWM(Motor speed/Speed)

enum MOTIONMODE
{
  LEFT,    /*left*/
  RIGHT,   /*right*/
  FORWARD, /*forward*/
  BACK,    /*backward*/
  STOP,    /*stop*/
  LEFT_FORWARD,
  LEFT_BACK,
  RIGHT_FORWARD,
  RIGHT_BACK,
};

extern enum MOTIONMODE mov_mode;

extern unsigned int carSpeed_rocker;

/*
  Control motor：Car movement forward
*/
void forward(int16_t in_carSpeed);

/*
  Control motor：Car moving backwards
*/
void backward(int16_t in_carSpeed);

/*
  Control motor：The car turns left and moves forward
*/
void left(int16_t in_carSpeed);

/*
  Control motor：The car turns right and moves forward
*/
void right(int16_t in_carSpeed);

/*
  Stop motor control：Turn off the motor drive
*/
void stop();

void forward_left(int16_t in_carSpeed);

void forward_right(int16_t in_carSpeed);

void back_left(int16_t in_carSpeed);

void back_right(int16_t in_carSpeed);
