#include "line_tracking.h"
#include "bluetooth.h"
#include "infrared.h"
#include "motor.h"
#include "utility.h"

unsigned long LT_PreMillis = 0;

void line_tracking_mode(void)
{
    if (func_mode == LineTracking)
    {
        if (LineTracking_Read_Middle)
        { //Detecting in the middle infrared tube

            forward(180); //Control motor：the car moving forward
            LT_PreMillis = millis();
        }
        else if (LineTracking_Read_Right)
        { //Detecting in the right infrared tube

            right(180); //Control motor：the car moving right
            while (LineTracking_Read_Right)
            {
                getBTData_Plus(); //Bluetooth data acquisition
                getIRData();      //Infrared data acquisition
            }
            LT_PreMillis = millis();
        }
        else if (LineTracking_Read_Left)
        {              //Detecting in the left infrared tube
            left(180); //Control motor：the car moving left
            while (LineTracking_Read_Left)
            {
                getBTData_Plus(); //Bluetooth data acquisition
                getIRData();      //Infrared data acquisition
            }
            LT_PreMillis = millis();
        }
        else
        {
            if (millis() - LT_PreMillis > 150)
            {
                stop(); //Stop motor control：Turn off motor drive mode
            }
        }
    }
}
