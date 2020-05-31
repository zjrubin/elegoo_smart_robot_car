#include "servo.h"

#include "bluetooth.h"
#include "infrared.h"

#include <Servo.h>

Servo servo; //  Create a DC motor drive object

void ServoControl(uint8_t angleSetting)
{
    if (angleSetting > 175)
    {
        angleSetting = 175;
    }
    else if (angleSetting < 5)
    {
        angleSetting = 5;
    }
    servo.attach(3);
    servo.write(angleSetting); //sets the servo position according to the  value
    delays(500);
    servo.detach();
}

void delays(unsigned long t)
{
    for (unsigned long i = 0; i < t; i++)
    {
        getBTData_Plus(); //Bluetooth Communication Data Acquisition
        getIRData();      //Infrared Communication Data Acquisition
        delay(1);
    }
}
