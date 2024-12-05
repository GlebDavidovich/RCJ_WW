#pragma once
#include <Arduino.h>
// #include "AirDebug.h"


struct OmniCamBlobInfo_t {
    int left_angle = 0;
    int right_angle = 0;
    int center_angle = 0;
    int width = 0;
    int clos_angle = 0;
    int distance = 0;
    int height = 0;
};
struct OmniCamData_t{
    OmniCamBlobInfo_t gates[2];
    //OmniCamBlobInfo_t blueGate;
};

extern OmniCamData_t camDataOmni;

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
OmniCamData_t omnicam();
void parseOmniCamData(char *data, int len);
bool getOpenMVDataOmni();
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
