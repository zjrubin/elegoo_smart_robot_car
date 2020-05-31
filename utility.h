#pragma once

enum SERIAL_mode
{
    Serial_rocker,
    Serial_programming,
    Serial_CMD,
};

extern enum SERIAL_mode Serial_mode;

enum FUNCTIONMODE
{
    IDLE,                  /*free*/
    LineTracking,          /*Line Tracking Mode*/
    ObstaclesAvoidance,    /*Obstacles Avoidance Mode*/
    Bluetooth,             /*Bluetooth Control Mode*/
    IRremote,              /*Infrared Control Mode*/
    CMD_MotorControl,      /*Motor Control Mode*/
    CMD_CarControl,        /*Car Control Mode*/
    CMD_CarControlxxx,     /*Car Control Mode*/
    CMD_ClearAllFunctions, /*Clear All Functions Mode*/
};

extern enum FUNCTIONMODE func_mode;
