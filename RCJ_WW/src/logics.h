#include "motors.h"
#include "sensors.h"

int goodAngle(int angle);

void projectSpeedOnLineXY(int speedX, int speedY, float lineX, float lineY, float &resSpeedX, float &resSpeedY);

void projectSpeedOnLine(float speed, float moveAngle, float lineX, float lineY, float &resSpeedX, float &resSpeedY);

void drive2ballSimple();

void move2gateSimple(int color);

void playForwardSimple(int color);

void playGoalkeeperFollowLine();

void playGoalkeeperOld();

void drive2ballOmni();

void move2gateOmni(int color);

void goOverObstacleOmni(float generalSpeed, float generalAngle, OmniCamBlobInfo_t obstacle, float critDist, bool rotate);

void playForwardOmni(int color);
