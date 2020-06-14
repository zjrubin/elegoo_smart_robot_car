#include "ultrasonic.h"

#include <Arduino.h>

const int ObstacleDetection = 35;

/*ULTRASONIC*/
unsigned int getDistance(void)
{ //Getting distance
    // static unsigned int tempda = 0;
    unsigned int tempda_x = 0;
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    tempda_x = ((unsigned int)pulseIn(ECHO_PIN, HIGH) / 58);
    // tempda = tempda_x;

    if (tempda_x > 50)
    {
        tempda_x = 50;
    }

    // return tempda;
    return tempda_x;
}
