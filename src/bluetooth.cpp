#include "ArduinoJson-v6.11.1.h" // Use ArduinoJson Libraries
#include "bluetooth.h"
#include "infrared.h"
#include "line_tracking.h"
#include "motor.h"
#include "servo.h"
#include "ultrasonic.h"
#include "utility.h"

#include <Arduino.h>

String CommandSerialNumber;

uint8_t CMD_MotorSelection;
uint8_t CMD_MotorDirection;

uint16_t CMD_MotorSpeed;
unsigned long CMD_leftMotorControl_Millis;
unsigned long CMD_rightMotorControl_Millis;

uint8_t CMD_CarDirection;
uint8_t CMD_CarSpeed;
uint16_t CMD_CarTimer;
unsigned long CMD_CarControl_Millis;

uint8_t CMD_CarDirectionxxx;
uint8_t CMD_CarSpeedxxx;
uint16_t CMD_Distance;

void bluetooth_mode()
{
    if (func_mode == Bluetooth)
    {
        switch (mov_mode)
        {
        case LEFT:
            left(carSpeed_rocker);
            break;
        case RIGHT:
            right(carSpeed_rocker);
            break;
        case FORWARD:
            forward(carSpeed_rocker);
            break;
        case BACK:
            backward(carSpeed_rocker);
            break;
        case STOP:
            stop();
            break;
        case LEFT_FORWARD:
            forward_left(carSpeed_rocker);
            break;
        case LEFT_BACK:
            back_left(carSpeed_rocker);
            break;
        case RIGHT_FORWARD:
            forward_right(carSpeed_rocker);
            break;
        case RIGHT_BACK:
            back_right(carSpeed_rocker);
            break;
        default:
            break;
        }
    }
}

/*****************************************************Begin@CMD**************************************************************************************/
/*
  N21:command
  CMD mode：Ultrasonic module：App controls module status, module sends data to app
*/
void CMD_UltrasoundModuleStatus_Plus(uint8_t is_get) //Ultrasonic module processing
{
    //uint16_t
    CMD_Distance = getDistance(); //Ultrasonic module measuring distance

    if (1 == is_get) // is_get Start  true：Obstacles / false:No obstacles
    {
        if (CMD_Distance <= 50)
        {
            Serial.print('{' + CommandSerialNumber + "_true}");
        }
        else
        {
            Serial.print('{' + CommandSerialNumber + "_false}");
        }
    }
    else if (2 == is_get) //Ultrasonic is_get data
    {
        char toString[10];
        sprintf(toString, "%d", CMD_Distance);
        // Serial.print(toString);
        Serial.print('{' + CommandSerialNumber + '_' + toString + '}');
    }
}
/*
  N22:command
   CMD mode：Tracking module：App controls module status, module sends data to app
*/
void CMD_TraceModuleStatus_Plus(uint8_t is_get) //Tracking module processing
{
    if (0 == is_get) /*Get traces on the left*/
    {
        if (LineTracking_Read_Left)
        {
            //Serial.print("{true}");
            Serial.print('{' + CommandSerialNumber + "_true}");
        }
        else
        {
            //Serial.print("{false}");
            Serial.print('{' + CommandSerialNumber + "_false}");
        }
    }
    else if (1 == is_get) /*Get traces on the middle*/
    {
        if (LineTracking_Read_Middle)
        {
            //Serial.print("{true}");
            Serial.print('{' + CommandSerialNumber + "_true}");
        }
        else
        {
            //Serial.print("{false}");
            Serial.print('{' + CommandSerialNumber + "_false}");
        }
    }
    else if (2 == is_get)
    { /*Get traces on the right*/

        if (LineTracking_Read_Right)
        {
            //Serial.print("{true}");
            Serial.print('{' + CommandSerialNumber + "_true}");
        }
        else
        {
            //Serial.print("{false}");
            Serial.print('{' + CommandSerialNumber + "_false}");
        }
    }
}

/*
  N1:command
  CMD mode：Sport mode <motor control> Control motor by app
  Input：uint8_t is_MotorSelection,  Motor selection   1：left  2：right  0：all
        uint8_t is_MotorDirection,   Motor steering  1：Forward  2：Reverse 0：stop
        uint8_t is_MotorSpeed,       Motor speed   0-250
*/
void CMD_MotorControl_Plus(uint8_t is_MotorSelection, uint8_t is_MotorDirection, uint8_t is_MotorSpeed)
{
    static boolean MotorControl = false;

    if (func_mode == CMD_MotorControl) //Motor control mode
    {
        MotorControl = true;
        if (is_MotorSelection == 1 || is_MotorSelection == 0) //Left motor
        {
            if (is_MotorDirection == 1) //Positive rotation
            {
                analogWrite(ENA, is_MotorSpeed);
                digitalWrite(IN1, HIGH);
                digitalWrite(IN2, LOW);
            }
            else if (is_MotorDirection == 2) //Reverse
            {
                analogWrite(ENA, is_MotorSpeed);
                digitalWrite(IN1, LOW);
                digitalWrite(IN2, HIGH);
            }
            else if (is_MotorDirection == 0)
            {
                digitalWrite(ENA, LOW); //Turn off the motor enable pin
            }
        }
        if (is_MotorSelection == 2 || is_MotorSelection == 0) //Right motor
        {
            if (is_MotorDirection == 1) //Positive rotation
            {
                analogWrite(ENB, is_MotorSpeed);
                digitalWrite(IN3, LOW);
                digitalWrite(IN4, HIGH);
            }
            else if (is_MotorDirection == 2) //Reverse
            {
                analogWrite(ENB, is_MotorSpeed);
                digitalWrite(IN3, HIGH);
                digitalWrite(IN4, LOW);
            }
            else if (is_MotorDirection == 0)
            {
                digitalWrite(ENB, LOW); //Turn off the motor enable pin
            }
        }
    }
    else
    {
        if (MotorControl == true)
        {
            MotorControl = false;
            digitalWrite(ENA, LOW); //Turn off the motor enable pin
            digitalWrite(ENB, LOW);
        }
    }
}

/*
  N4：command
  CMD mode：<Car control> APP control car
  Time limited
*/
void CMD_CarControl_Plus(uint8_t is_CarDirection, uint8_t is_CarSpeed, uint8_t is_Timer)
{
    static boolean CarControl = false;
    static boolean CarControl_TE = false; //Have time to spend
    static boolean CarControl_return = false;
    if (func_mode == CMD_CarControl) //Car Control Mode
    {
        CarControl = true;
        if (is_Timer != 0) //Setting time cannot be empty
        {
            if ((millis() - CMD_CarControl_Millis) > (is_Timer * 1000)) //check the time
            {
                CarControl_TE = true;
                digitalWrite(ENA, LOW); //Turn off the motor enable pin
                digitalWrite(ENB, LOW);
                if (CarControl_return == false)
                {
                    Serial.print('{' + CommandSerialNumber + "_ok}");
                    delay(1);
                    Serial.print('{' + CommandSerialNumber + "_ok}");
                    CarControl_return = true;
                }
            }
            else
            {
                CarControl_TE = false; //Have time to spend
                CarControl_return = false;
            }
        }
        if (CarControl_TE == false)
        {
            switch (is_CarDirection)
            {
            case 1: /*Left-Forward Motion Mode*/
                left(is_CarSpeed);
                break;
            case 2: /*Right-Forward Motion Mode*/
                right(is_CarSpeed);
                break;
            case 3: /*Sport mode forward*/
                forward(is_CarSpeed);
                break;
            case 4: /*Sport mode backward*/
                backward(is_CarSpeed);
                break;
            default:
                break;
            }
        }
    }
    else
    {
        if (CarControl == true)
        {
            CarControl_return = false;
            CarControl = false;
            digitalWrite(ENA, LOW); //Turn off the motor enable pin
            digitalWrite(ENB, LOW);
            CMD_CarControl_Millis = 0;
        }
    }
}

/*
  N40：command
  CMD mode：<Car control> APP control car
  No time limit
*/
void CMD_CarControl_Plusxxx(uint8_t is_CarDirection, uint8_t is_CarSpeed)
{
    static boolean CarControl = false;
    if (func_mode == CMD_CarControlxxx) //Car Control Mode
    {
        CarControl = true;
        switch (is_CarDirection)
        {
        case 1: /*Left-Forward Motion Mode*/
            left(is_CarSpeed);
            break;
        case 2: /*Right-Forward Motion Mode*/
            right(is_CarSpeed);
            break;
        case 3: /*Sport mode forward*/
            forward(is_CarSpeed);
            break;
        case 4: /*Sport mode backward*/
            backward(is_CarSpeed);
            break;
        default:
            break;
        }
    }
    else
    {
        if (CarControl == true)
        {
            CarControl = false;
            digitalWrite(ENA, LOW); //Turn off the motor enable pin
            digitalWrite(ENB, LOW);
        }
    }
}

/*
  N5:command
  CMD mode：
*/
void CMD_ClearAllFunctionsXXX(void)
{
    if (func_mode == CMD_ClearAllFunctions)
    {

        mov_mode = STOP;
        func_mode = IDLE;
        digitalWrite(ENA, LOW); //Turn off the motor enable pin
        digitalWrite(ENB, LOW);

        /*CMD_MotorControl:Motor Control： Motor Speed、Motor Direction、Motor Time*/
        CMD_MotorSelection = 0;
        CMD_MotorDirection = 0;

        CMD_MotorSpeed = 0;
        CMD_leftMotorControl_Millis = 0;
        CMD_rightMotorControl_Millis = 0;

        /*CMD_CarControl:Car Control：Car moving direction、Car Speed、Car moving time*/
        CMD_CarDirection = 0;
        CMD_CarSpeed = 0;
        CMD_CarTimer = 0;
        CMD_CarControl_Millis = 0;
    }
}

void getDistance_xx(void)
{
    CMD_Distance = getDistance(); //Ultrasonic measurement distance
}

/*****************************************************End@CMD**************************************************************************************/
/*
  Bluetooth serial port data acquisition and control command parsing
*/
void getBTData_Plus(void)
{
    static String SerialPortData = "";
    uint8_t c = 0;
    if (Serial.available() > 0)
    {
        while ((c != '}') && Serial.available() > 0) //Forcibly wait for a frame of data to finish receiving
        {
            // while (Serial.available() == 0)
            //   ;
            c = Serial.read();
            SerialPortData += (char)c;
        }
    }
    if (c == '}')
    {
        //Serial.println(SerialPortData);
        StaticJsonDocument<200> doc;                                       //Create a JsonDocument object
        DeserializationError error = deserializeJson(doc, SerialPortData); //Deserialize JSON data
        SerialPortData = "";
        if (!error) //Check if deserialization is successful
        {
            int control_mode_N = doc["N"];
            char buf[3];
            uint8_t temp = doc["H"];
            sprintf(buf, "%d", temp);
            CommandSerialNumber = buf; //Get the serial number of the new command
            switch (control_mode_N)
            {
            case 1: /*Motion module  processing <command：N 1>*/
            {
                Serial_mode = Serial_programming;
                func_mode = CMD_MotorControl;
                CMD_MotorSelection = doc["D1"];
                CMD_MotorDirection = doc["D2"];
                CMD_MotorSpeed = doc["D3"];
                Serial.print('{' + CommandSerialNumber + "_ok}");
            }
            break;
            case 2: /*Remote switching mode  processing <command：N 2>*/
            {
                Serial_mode = Serial_rocker;
                int SpeedRocker = doc["D2"];
                if (SpeedRocker != 0)
                {
                    carSpeed_rocker = SpeedRocker;
                }
                if (1 == doc["D1"])
                {
                    func_mode = Bluetooth;
                    mov_mode = LEFT;
                    // Serial.print('{' + CommandSerialNumber + "_ok}");
                }
                else if (2 == doc["D1"])
                {
                    func_mode = Bluetooth;
                    mov_mode = RIGHT;
                    // Serial.print('{' + CommandSerialNumber + "_ok}");
                }
                else if (3 == doc["D1"])
                {
                    func_mode = Bluetooth;
                    mov_mode = FORWARD;
                    //Serial.print('{' + CommandSerialNumber + "_ok}");
                }
                else if (4 == doc["D1"])
                {
                    func_mode = Bluetooth;
                    mov_mode = BACK;
                    //Serial.print('{' + CommandSerialNumber + "_ok}");
                }
                else if (5 == doc["D1"])
                {
                    func_mode = Bluetooth;
                    mov_mode = STOP;
                    //Serial.print('{' + CommandSerialNumber + "_ok}");
                }
                else if (6 == doc["D1"])
                {
                    func_mode = Bluetooth;
                    mov_mode = LEFT_FORWARD;
                    //Serial.print('{' + CommandSerialNumber + "_ok}");
                }
                else if (7 == doc["D1"])
                {
                    func_mode = Bluetooth;
                    mov_mode = LEFT_BACK;
                    //Serial.print('{' + CommandSerialNumber + "_ok}");
                }
                else if (8 == doc["D1"])
                {
                    func_mode = Bluetooth;
                    mov_mode = RIGHT_FORWARD;
                    //Serial.print('{' + CommandSerialNumber + "_ok}");
                }
                else if (9 == doc["D1"])
                {
                    func_mode = Bluetooth;
                    mov_mode = RIGHT_BACK;
                    //Serial.print('{' + CommandSerialNumber + "_ok}");
                }
            }
            break;
            case 3: /*Remote switching mode  processing <command：N 3>*/
            {
                Serial_mode = Serial_rocker;
                if (1 == doc["D1"]) // Line Tracking Mode
                {
                    func_mode = LineTracking;
                    Serial.print('{' + CommandSerialNumber + "_ok}");
                }
                else if (2 == doc["D1"]) //Obstacles Avoidance Mode
                {
                    func_mode = ObstaclesAvoidance;
                    Serial.print('{' + CommandSerialNumber + "_ok}");
                }
            }
            break;
            case 4: /*Motion module  processing <command：N 4>*/
            {
                Serial_mode = Serial_programming;
                func_mode = CMD_CarControl;
                CMD_CarDirection = doc["D1"];
                CMD_CarSpeed = doc["D2"];
                CMD_CarTimer = doc["T"];
                CMD_CarControl_Millis = millis(); //Get the timestamp
                                                  //Serial.print("{ok}");
            }
            break;
            case 5: /*Clear mode  processing <command：N 5>*/
            {
                func_mode = CMD_ClearAllFunctions;
                Serial.print('{' + CommandSerialNumber + "_ok}");
            }

            break;
            case 6: /*CMD mode：angle Setting*/
            {
                uint8_t angleSetting = doc["D1"];
                ServoControl(angleSetting);
                Serial.print('{' + CommandSerialNumber + "_ok}");
            }

            break;
            case 21: /*Ultrasonic module  processing <command：N 21>*/
            {
                Serial_mode = Serial_programming;
                CMD_UltrasoundModuleStatus_Plus(doc["D1"]);
            }

            break;
            case 22: /*Trace module data processing <command：N 22>*/
            {
                Serial_mode = Serial_programming;
                CMD_TraceModuleStatus_Plus(doc["D1"]);
            }
            break;
            case 40:
            {
                Serial_mode = Serial_programming;
                func_mode = CMD_CarControlxxx;
                CMD_CarDirectionxxx = doc["D1"];
                CMD_CarSpeedxxx = doc["D2"];
                Serial.print('{' + CommandSerialNumber + "_ok}");
            }
            break;
            default:
                break;
            }
        }
    }
    else if (SerialPortData != "")
    {
        if (true == SerialPortData.equals("f"))
        {
            func_mode = CMD_CarControlxxx;
            CMD_CarDirectionxxx = 3;
            CMD_CarSpeedxxx = 180;
            SerialPortData = "";
        }
        else if (true == SerialPortData.equals("b"))
        {
            func_mode = CMD_CarControlxxx;
            CMD_CarDirectionxxx = 4;
            CMD_CarSpeedxxx = 180;
            SerialPortData = "";
        }
        else if (true == SerialPortData.equals("l"))
        {
            func_mode = CMD_CarControlxxx;
            CMD_CarDirectionxxx = 1;
            CMD_CarSpeedxxx = 180;
            SerialPortData = "";
        }
        else if (true == SerialPortData.equals("r"))
        {
            func_mode = CMD_CarControlxxx;
            CMD_CarDirectionxxx = 2;
            CMD_CarSpeedxxx = 180;
            SerialPortData = "";
        }
        else if (true == SerialPortData.equals("s"))
        {
            func_mode = Bluetooth;
            mov_mode = STOP;
            SerialPortData = "";
        }
        else if (true == SerialPortData.equals("1"))
        {
            func_mode = LineTracking;
            SerialPortData = "";
        }
        else if (true == SerialPortData.equals("2"))
        {
            func_mode = ObstaclesAvoidance;
            SerialPortData = "";
        }
    }
}
