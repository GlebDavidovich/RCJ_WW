#pragma once
#include <Arduino.h>

struct OmniCamBlobInfo_t
{
    uint16_t leftAngle = 0;
    uint16_t rightAngle = 0;
    uint16_t center = 0;
    uint16_t width = 0;
    uint16_t distance = 0;
    uint16_t closest = 0;
};

void mpuAndCamInit();
void setupOffsetsFromEEPROM();
void meansensors();
void calibration();
int16_t mpuGetDegree();
int getStrength();
int getBallAngle();
int getBallAngleGoalkeeper();
bool isBall();
bool isBallGoalkeeper();
int8_t getCamData(int color);
int getCamHeight();
uint8_t OpenMV_ReadData(uint8_t cam_id, uint8_t addr, uint8_t len, uint8_t *resp);
void getOpenMVCamData(int color);
OmniCamBlobInfo_t getBlueGate();
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
int normalizedMux(int sensor);
void saveLineDirection();
void getLineDirection_Delayed(float& x, float& y);
int getLineAngle_Delayed();
int getLineTime(int index);
