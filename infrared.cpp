#include "infrared.h"
#include "motor.h"
#include "utility.h"

// MIGHT HAVE TO DEFINE ARDUINO_DEBUG BEFORE THIS INCLUDE
#include "arduino_debug.h"

IRrecv irrecv(RECV_PIN); //  Create an infrared receive drive object
decode_results results;  //  Create decoding object

unsigned long IR_PreMillis;

void getIRData(void)
{
    if (irrecv.decode(&results))
    {
        unsigned long new_millis = millis();
        debug_serial_println(new_millis - IR_PreMillis);
        IR_PreMillis = new_millis;
        switch (results.value)
        {
        case IR_FORWARD:
        case IR_FORWARD_UNKNOWN:
            func_mode = IRremote;
            mov_mode = FORWARD;
            break; /*forward*/
        case IR_BACKWARD:
        case IR_BACKWARD_UNKNOWN:
            func_mode = IRremote;
            mov_mode = BACK;
            break; /*backward*/
        case IR_LEFT:
        case IR_LEFT_UNKNOWN:
            func_mode = IRremote;
            mov_mode = LEFT;
            break; /*left*/
        case IR_RIGHT:
        case IR_RIGHT_UNKNOWN:
            func_mode = IRremote;
            mov_mode = RIGHT;
            break; /*right*/
        case IR_STOP:
        case IR_STOP_UNKNOWN:
            func_mode = IRremote;
            mov_mode = STOP;
            break; /*stop*/
        case IR_ONE:
        case IR_ONE_UNKNOWN:
            func_mode = LineTracking;
            break; /*Line Tracking Mode*/
        case IR_TWO:
        case IR_TWO_UNKNOWN:
            func_mode = ObstaclesAvoidance;
            break; /*Obstacles Avoidance Mode*/
        case IR_REPEAT:
            // Keep the same function mode
            func_mode = func_mode;
            break;
        default:
            break;
        }
        irrecv.resume();
    }
    else
    {
        if (func_mode == IRremote && (millis() - IR_PreMillis) > IR_TIMEOUT)
        {
            // Send a stop code
            mov_mode = STOP;
        }
    }
}

void irremote_mode(void)
{
    if (func_mode == IRremote)
    {
        switch (mov_mode)
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
        // if (millis() - IR_PreMillis > 500)
        // {
        //     mov_mode = STOP;
        //     IR_PreMillis = millis();
        // }
    }
}
