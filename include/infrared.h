#pragma once

#include <IRremote.h>

/*Arduino pin is connected to the IR Receiver*/
#define RECV_PIN 12

#define IR_FORWARD 0xFF629D
#define IR_FORWARD_UNKNOWN 0x511DBB

#define IR_BACKWARD 0xFFA857
#define IR_BACKWARD_UNKNOWN 0xA3C8EDDB

#define IR_LEFT 0xFF22DD
#define IR_LEFT_UNKNOWN 0x52A3D41F

#define IR_RIGHT 0xFFC23D
#define IR_RIGHT_UNKNOWN 0x20FE4DBB

#define IR_STOP 0xFF02FD
#define IR_STOP_UNKNOWN 0xD7E84B1B

#define IR_ONE 0xFF6897 // Line Tracking mode
#define IR_ONE_UNKNOWN 0xC101E57B

#define IR_TWO 0xFF9867 // Obstacles Avoidance mode
#define IR_TWO_UNKNOWN 0x97483BFB

#define IR_ASTERISK 0xFF42BD
#define IR_HASH 0xFF52AD

#define IR_REPEAT 0xFFFFFFFF

// If no IR code is received within the (150 millisecond) timeout window,
// Set movement to stop
// NOTE: the timeout period (150 milliseconds) is subject to change.
// with the current code, it depends on how long the whole code takes to complete
// its superloop. CHANGE this timeout period when the code is refactored
// A smaller timeout period is better as it translates less latency between
// Releasing a IR button and the car no longer moving according to that button
#define IR_TIMEOUT 100

extern IRrecv irrecv;          //  Create an infrared receive drive object
extern decode_results results; //  Create decoding object

extern unsigned long IR_PreMillis;

/*
  Infrared Communication Data Acquisition
*/
void getIRData(void);

/*
  Infrared remote control mode
*/
void irremote_mode(void);
