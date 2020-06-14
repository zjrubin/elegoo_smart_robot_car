
#include "bluetooth.h"
#include "infrared.h"
#include "line_tracking.h"
#include "motor.h"
#include "servo.h"
#include "ultrasonic.h"
#include "utility.h"

#define ARDUINO_DEBUG
#include "arduino_debug.h"

#include <Arduino.h>

#define BAUD_RATE 115200

#define LED_Pin 13

int rightDistance = 0;  //Right distance
int leftDistance = 0;   //left Distance
int middleDistance = 0; //middle Distance

void obstacles_avoidance_mode(void);

void setup(void)
{
    Serial.begin(BAUD_RATE); //initialization
    ServoControl(90);
    irrecv.enableIRIn(); //Enable infrared communication NEC

    pinMode(ECHO_PIN, INPUT); //Ultrasonic module initialization
    pinMode(TRIG_PIN, OUTPUT);

    pinMode(IN1, OUTPUT); //Motor-driven port configuration
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    pinMode(LineTracking_Pin_Right, INPUT); //Infrared tracking module port configuration
    pinMode(LineTracking_Pin_Middle, INPUT);
    pinMode(LineTracking_Pin_Left, INPUT);
}

void loop(void)
{
    getBTData_Plus();           //Bluetooth data acquisition
    getIRData();                //Infrared data acquisition
    bluetooth_mode();           //Bluetooth remote mode
    irremote_mode();            //Infrared NEC remote control mode
    line_tracking_mode();       //Line Tracking Mode
    obstacles_avoidance_mode(); //Obstacles Avoidance Mode

    CMD_Distance = getDistance(); // Ultrasonic measurement distance
    /*CMD_MotorControl: Motor Control： Motor Speed、Motor Direction、Motor Time*/
    CMD_MotorControl_Plus(CMD_MotorSelection, CMD_MotorDirection, CMD_MotorSpeed); //Control motor steering
    /*  CMD mode：<Car control> APP control car*/
    CMD_CarControl_Plus(CMD_CarDirection, CMD_CarSpeed, CMD_CarTimer); //Control the direction of the car<Time limited>
    CMD_CarControl_Plusxxx(CMD_CarDirectionxxx, CMD_CarSpeedxxx);      //Control the direction of the car<No Time limited>
    CMD_ClearAllFunctionsXXX();
}

/*f(x) int */
static boolean function_xxx(long xd, long sd, long ed) //f(x)
{
    if (sd <= xd && xd <= ed)
        return true;
    else
        return false;
}

/*Obstacle avoidance*/

void obstacles_avoidance_mode(void)
{
    static boolean first_is = true;
    uint8_t switc_ctrl = 0;
    if (func_mode == ObstaclesAvoidance)
    {
        if (first_is == true) //Enter the mode for the first time, and modulate the steering gear to 90 degrees
        {
            ServoControl(90);
            first_is = false;
        }
        uint8_t get_Distance = getDistance();
        if (function_xxx(get_Distance, 0, 20))
        {
            stop();
            /*
      ------------------------------------------------------------------------------------------------------
      ServoControl(30 * 1): 0 1 0 1 0 1 0 1
      ServoControl(30 * 3): 0 0 1 1 0 0 1 1
      ServoControl(30 * 5): 0 0 0 0 1 1 1 1
      1 2 4 >>>             0 1 2 3 4 5 6 7
      1 3 5 >>>             0 1 3 4 5 6 5 9
      ------------------------------------------------------------------------------------------------------
      Truth table of obstacle avoidance state
      */
            for (int i = 1; i < 6; i += 2) //1、3、5 Omnidirectional detection of obstacle avoidance status
            {
                ServoControl(30 * i);
                get_Distance = getDistance();
                delays(200);
                if (function_xxx(get_Distance, 0, 5))
                {
                    switc_ctrl = 10;
                    break;
                }
                else if (function_xxx(get_Distance, 0, 20)) //How many cm in the front have obstacles?
                {
                    switc_ctrl += i;
                }
            }
            ServoControl(90);
        }
        else if (function_xxx(get_Distance, 20, 50))
        {
            forward(150); //Control car forwar
        }
        while (switc_ctrl)
        {
            switch (switc_ctrl)
            {
            case 1:
            case 5:
            case 6:
                forward(150); //Control car forwar
                switc_ctrl = 0;
                break;
            case 3:
                left(250); //Control car left
                switc_ctrl = 0;
                break;
            case 4:
                left(250); //Control car left
                switc_ctrl = 0;
                break;
            case 8:
            case 11:
                right(250); //Control car right
                switc_ctrl = 0;
                break;
            case 9:
            case 10:
                backward(150); //Control car Car backwards
                switc_ctrl = 11;
                break;
            }
            ServoControl(90);
        }
    }
    else
    {
        first_is = true;
    }
}
