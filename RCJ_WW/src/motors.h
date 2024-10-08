#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>

void motorsPins();
void singleMotorControl(uint8_t num, int16_t speed);
void drive(int16_t m1, int16_t m2, int16_t m3, int16_t m4);
void drive(float angle, int rotation_speed, int speed);
void drive(float angle, int speed);
void kick();
void turnByDegree(int16_t degree);
void drive2ball();
void move2gate();
void dribble(uint8_t speed);