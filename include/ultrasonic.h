#pragma once

/*Arduino pin is connected to the Ultrasonic sensor module*/
#define ECHO_PIN A4
#define TRIG_PIN A5
extern const int ObstacleDetection;

unsigned int getDistance(void);
