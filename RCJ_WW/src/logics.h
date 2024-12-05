#pragma once
#include "motors.h"
#include "sensors.h"

int goodAngle(int angle);

void projectSpeedOnLineXY(int speedX, int speedY, float lineX, float lineY, float &resSpeedX, float &resSpeedY);

void projectSpeedOnLine(float speed, float moveAngle, float lineX, float lineY, float &resSpeedX, float &resSpeedY);

void playForwardGoyda(int color);

void playForwardDribble(int color);

void make_pause(int ms);

void playForwardDribble2(int color);

void goalRotate(int color);

void goalDriveBack(int color);

void killerFeature(int color);

void playGoalkeeperCamera(int color);

void playGoalkeeperSuperlast(int color);

void drive2ballSimple();

void move2gateSimple(int color);

void playForwardSimple(int color);

void set_kp(int value);

void playGoalkeeperFollowLine();

void playGoalkeeperGoyda(int color);

void dribblerGoalGoyda(int color);

void playGoalkeeperOld();

void drive2ballOmni();

void move2gateOmni(int color);

void goOverObstacleOmni(float generalSpeed, float generalAngle, OmniCamBlobInfo_t obstacle, float critDist, bool rotate);

void playForwardOmni(int color);
