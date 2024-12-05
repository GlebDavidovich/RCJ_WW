#define DebugInfo false

#pragma once
#include "sensors.h"

#if DebugInfo
#include <Arduino.h>
#include <BluetoothSerial.h>
extern BluetoothSerial BT;

// int x____ = camDataOmni.gates[0].center_angle;

class AirDebug
{
public:
    uint8_t outbuf [40];

    int16_t mask1 = 0b1111111100000000;
    int16_t mask2 = 0b0000000011111111;


    bool line_sensor[16];
    int16_t line_angle = 0;

    int16_t ball_angle = 0;

    int16_t gyroskope_angle = 0;

    int8_t speed_rotate = 0;
    int8_t speed_x = 0;
    int8_t speed_y = 0;

    uint8_t dribl_speed = 0;

    uint8_t state;
    bool is_ball = false;
public:
    void SendInfo();
    void SetData();
    
};

extern AirDebug Debug;
#endif