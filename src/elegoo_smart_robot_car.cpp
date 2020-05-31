
#include "arduino_debug.h"
#include "bluetooth.h"
#include "infrared.h"
#include "line_tracking.h"
#include "motor.h"
#include "servo.h"
#include "ultrasonic.h"
#include "utility.h"

#include <Arduino.h>

#include <Arduino_FreeRTOS.h>
#include <queue.h>

QueueHandle_t g_movement_queue;

void task_move(void *pvParameters);
void task_get_infrared_movement(void *pvParameters);
void obstacles_avoidance_mode(void);

void setup(void)
{

    g_movement_queue = xQueueCreate(10, sizeof(enum MOTIONMODE));

    if (g_movement_queue != NULL)
    {
        Serial.begin(BAUD_RATE); // initialization
        ServoControl(90);

        pinMode(ECHO_PIN, INPUT); // Ultrasonic module initialization
        pinMode(TRIG_PIN, OUTPUT);

        pinMode(LineTracking_Pin_Right, INPUT); //Infrared tracking module port configuration
        pinMode(LineTracking_Pin_Middle, INPUT);
        pinMode(LineTracking_Pin_Left, INPUT);

        xTaskCreate(task_get_infrared_movement, "get_infrared_movement",
                    configMINIMAL_STACK_SIZE, NULL, 1, NULL);

        xTaskCreate(task_move, "move",
                    configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    }
}

void loop(void)
{
    // debug_serial_println("idle task");
    // getBTData_Plus();           //Bluetooth data acquisition
    // getIRData(); //Infrared data acquisition
    // bluetooth_mode();           //Bluetooth remote mode
    // irremote_mode(); //Infrared NEC remote control mode
    // line_tracking_mode();       //Line Tracking Mode
    // obstacles_avoidance_mode(); //Obstacles Avoidance Mode

    // CMD_Distance = getDistance(); // Ultrasonic measurement distance
    /*CMD_MotorControl: Motor Control： Motor Speed、Motor Direction、Motor Time*/
    // CMD_MotorControl_Plus(CMD_MotorSelection, CMD_MotorDirection, CMD_MotorSpeed); //Control motor steering
    /*  CMD mode：<Car control> APP control car*/
    // CMD_CarControl_Plus(CMD_CarDirection, CMD_CarSpeed, CMD_CarTimer); //Control the direction of the car<Time limited>
    // CMD_CarControl_Plusxxx(CMD_CarDirectionxxx, CMD_CarSpeedxxx);      //Control the direction of the car<No Time limited>
    // CMD_ClearAllFunctionsXXX();
}

void task_move(void *pvParameters)
{
    // Motor-driven port configuration
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    for (;;)
    {
        enum MOTIONMODE movement;
        if (xQueueReceive(g_movement_queue, &movement, portMAX_DELAY) == pdPASS)
        {
            switch (movement)
            {
            case FORWARD:
                forward(carSpeed);
                break;
            case BACK:
                backward(carSpeed);
                break;
            case LEFT:
                left(carSpeed);
                break;
            case RIGHT:
                right(carSpeed);
                break;
            case STOP:
                stop();
                break;
            default:
                break;
            }
        }
    }
}

void task_get_infrared_movement(void *pvParameters)
{
    TickType_t old_tick_count = xTaskGetTickCount();
    enum MOTIONMODE movement = STOP;
    irrecv.enableIRIn(); // Enable infrared communication NEC

    for (;;)
    {
        TickType_t current_tick_count = xTaskGetTickCount();
        if (irrecv.decode(&results))
        {
            debug_serial_println((current_tick_count - old_tick_count) * portTICK_PERIOD_MS);
            old_tick_count = current_tick_count;

            switch (results.value)
            {
            case IR_FORWARD:
            case IR_FORWARD_UNKNOWN:
                func_mode = IRremote;
                movement = FORWARD;
                break;
            case IR_BACKWARD:
            case IR_BACKWARD_UNKNOWN:
                func_mode = IRremote;
                movement = BACK;
                break;
            case IR_LEFT:
            case IR_LEFT_UNKNOWN:
                func_mode = IRremote;
                movement = LEFT;
                break;
            case IR_RIGHT:
            case IR_RIGHT_UNKNOWN:
                func_mode = IRremote;
                movement = RIGHT;
                break;
            case IR_STOP:
            case IR_STOP_UNKNOWN:
                func_mode = IRremote;
                movement = STOP;
                break;
            case IR_ONE:
            case IR_ONE_UNKNOWN:
                func_mode = LineTracking;
                break; /*Line Tracking Mode*/
            case IR_TWO:
            case IR_TWO_UNKNOWN:
                func_mode = ObstaclesAvoidance;
                break; /*Obstacles Avoidance Mode*/
            case IR_REPEAT:
            default:
                // Keep the same movement that was previously stored
                break;
            }

            irrecv.resume();
        }
        else
        {
            if (func_mode == IRremote && ((current_tick_count - old_tick_count) * portTICK_PERIOD_MS) > IR_TIMEOUT)
            {
                // Send a stop code
                movement = STOP;
            }
        }

        xQueueSend(g_movement_queue, &movement, portMAX_DELAY);
    }
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
