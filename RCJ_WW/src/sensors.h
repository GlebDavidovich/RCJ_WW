#pragma once
#include <Arduino.h>

void mpuAndCamInit();
void setupOffsetsFromEEPROM();
void meansensors();
void calibration();
int16_t mpuGetDegree();
int getStrength();
int getAngle();
int getAngle_600();
bool isBall();
int8_t getCamData(char color);
int readMux(int channel);
void getGreen();
void whiteTo0();
void getWhite();
void EEPROMSaveLines();
int isLine();
bool isLineBehind();