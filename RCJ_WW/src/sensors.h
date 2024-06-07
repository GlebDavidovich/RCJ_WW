#pragma once
#include <Arduino.h>

void mpuAndCamInit();
void setupOffsetsFromEEPROM();
void meansensors();
void calibration();
int16_t mpuGetDegree();
int getStrength();
int getBallAngle();
int getBallAngle_600();
bool isBall();
int8_t getCamData(char color);
int readMux(int channel);
void getGreen();
void whiteTo0();
void getWhite();
void EEPROMSaveLines();
bool isLineOnSensor(int sensor);
int getLineAngle_Avg();
int getLineAngle_Max();
bool isLineBehind();
int getLastLineDirection();
bool setLastLineDirection(int value);
int getErr(int sensor);