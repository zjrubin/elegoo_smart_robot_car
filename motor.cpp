#include "motor.h"

#include "arduino_debug.h"

#include <Arduino.h>

enum MOTIONMODE mov_mode = STOP;

unsigned int carSpeed_rocker = 250;

void forward(int16_t in_carSpeed)
{
    analogWrite(ENA, in_carSpeed);
    analogWrite(ENB, in_carSpeed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    // debug_serial_println("forward");
}

void backward(int16_t in_carSpeed)
{
    analogWrite(ENA, in_carSpeed);
    analogWrite(ENB, in_carSpeed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    // debug_serial_println("backward");
}

void left(int16_t in_carSpeed)
{

    analogWrite(ENA, in_carSpeed);
    analogWrite(ENB, in_carSpeed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    // debug_serial_println("left");
}

void right(int16_t in_carSpeed)
{
    analogWrite(ENA, in_carSpeed);
    analogWrite(ENB, in_carSpeed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    // debug_serial_println("right");
}

void stop()
{
    digitalWrite(ENA, LOW);
    digitalWrite(ENB, LOW);

    // debug_serial_println("stop");
}

void forward_left(int16_t in_carSpeed)
{
    analogWrite(ENA, in_carSpeed / 2);
    analogWrite(ENB, in_carSpeed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    debug_serial_println("forward left");
}

void forward_right(int16_t in_carSpeed)
{
    analogWrite(ENA, in_carSpeed);
    analogWrite(ENB, in_carSpeed / 2);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    debug_serial_println("forward right");
}

void back_left(int16_t in_carSpeed)
{
    analogWrite(ENA, in_carSpeed / 2);
    analogWrite(ENB, in_carSpeed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    debug_serial_println("backward left");
}

void back_right(int16_t in_carSpeed)
{
    analogWrite(ENA, in_carSpeed);
    analogWrite(ENB, in_carSpeed / 2);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    debug_serial_println("backward right");
}
