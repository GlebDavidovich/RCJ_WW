#pragma once
#include "logics.h"

//[Header("Управляющие переменные")]
float timer = 0;
int stateGame = 0;

//[Header("Игровая логика")]
float moveAngle = 0;
float deltaAngle = 0;
int speed = 0;

int lineSpeed = 50;
float ballAngle = 0;
float lineAngle = 0;
float cameraAngle = 0;

float lastLineAngle = 0;
float lastLineTime = 0;
float lastlineReset = 0;

int speedX = 0;
int speedY = 0;

float isBallDiap = 10;
float isBallStrength = 120;

//[Header("Нападающий")]
int drive2ballSpeed = 50;
int move2gateSpeed = 60;
float goRoundBallDiap = 20;
float goRoundBallCoefFw = 1.f;
int timeBallHeld = 0;
int gateMovePause = 1000;
float goRoundObstacleCoef = 7.f;
float rotateObstacleCoef = 0.125f;

int dribblerSpeed = 50;

float fb_kp = 3;
float fb_ki = 0;
float fb_kd = 40;
float fwBallIntegral = 0;
float fwBallPrev = 0;

int maxTorqueWithBall = 20;
int maxSpeedBackWithBall = 70;

int criticalObstacleAngle = 150;
int criticalObstacleDistance = 4;

//[Header("Вратарь (простой)")]
int goalkeeperLineSpeed = 200;
int goalkeeperVertSpeed = 100;
float horizCoef = 180;
float mainSpeedY = 0;
int minSpeedY = -100;
float accelerationY = -2;

int timeToPush = 1000;
int stuckTime = 0;
int ballSide = 1;

//[Header("Вратарь (следование по линии)")]
float rotateSlowingCoef = 1;
float considerLineTime = 10;
float lineSensorPriority[16] = {0, 0.8, 0.2, 0.2, 0.2, 0.6, 0.8, 0, 0, 0.8, 0.6, 0.2, 0.2, 0.2, 0.8, 0};
int activeSensors = 8;
int maxGoalkeeperAngle = 80;
float goRoundBallCoefGk = 0.5f;

float g_kp = 50;
float g_ki = 0;
float g_kd = 30;

float leftIntegral = 0;
float leftPrev = 0;
float rightIntegral = 0;
float rightPrev = 0;

float gb_kp = 1;
float gb_ki = 0;
float gb_kd = 40;
float gkBallIntegral = 0;
float gkBallPrev = 0;

float gkReactOnBallDiap = 0;

// Вратарь по камере
int lastGateAngle = 180;
int lastGateTime = 0;
int lastGateReset = 5000;

float gate_kp = 0.5;
float gate_kp_neg = 0.5;
float gate_kd = 0;
float gate_ki = 0;

float gateIntegral = 0;
float gatePrev = 0;

float limitGateIntegral = 150;
float limitGateSpeed = 70;

float ballMoveTime = 0;
float lastBallAngle = 0;
float ballNoMotionDiap = 10;
float lastMoveBallStrength = 0;
float ballNoMotionStrength = 15;

float kfTimer = 0;
float maxKfTime = 10000;

float prevBallStrength = 0;
// float gk_st_kd = 100;

int goodAngle(int angle) {
    angle %= 360;
    if (angle < -180)
        angle += 360;
    if (angle > 180)
        angle -= 360;
    return angle;
}

void projectSpeedOnLineXY(int speedX, int speedY, float lineX, float lineY, float& resSpeedX, float& resSpeedY)
{
    if (lineX * speedX + lineY * speedY > 0)
    {
        lineX += lineY;
        lineY = lineX - lineY;
        lineX -= lineY;
        lineX = -lineX;
        float resSpeed = lineX * speedX + lineY * speedY;
        resSpeedX = resSpeed * lineX;
        resSpeedY = resSpeed * lineY;
    }
    else
    {
        resSpeedX = speedX;
        resSpeedY = speedY;
    }
}

void projectSpeedOnLine(float speed, float moveAngle, float lineX, float lineY, float& resSpeedX, float& resSpeedY)
{
    int speedX = (int)(speed * sin(moveAngle * DEG_TO_RAD));
    int speedY = (int)(speed * cos(moveAngle * DEG_TO_RAD));
    projectSpeedOnLineXY(speedX, speedY, lineX, lineY, resSpeedX, resSpeedY);
}

void playForwardGoyda(int color){
  while (true){
    getOpenMVDataOmni();
    saveLineDirection();
    // if (omnicam().gates[1].width == 0){
    //   dribble(20);
    // }
    // else{
    //   dribble(0);
    // }

    int cam_angle = -omnicam().gates[color].center_angle;
    int cam_dist = omnicam().gates[color].distance;
    
    int st = getStrength();
    int ballAngle = getBallAngle();
    int lineAngle = getLineAngle_Delayed();

    if (abs(mpuGetDegree()) > 80){
        drive(0, -mpuGetDegree() * 0.3, 0);
    }

    else if (!isBall()){
        //dribble((abs(ballAngle) < 40) ? 30 : 0);
        int deltaAngle = -mpuGetDegree(); //ballAngle * 0.3;
        if (lineAngle == 360){
            moveAngle = ballAngle + (ballAngle > 0) ? 15 : -15;
            if (abs(ballAngle) < 25){
                moveAngle = ballAngle;
            }
            double k = 0.5;

            int delta = 0;
            // if (abs(angle) <= 45){
            //   k = 3;
            // }
            if (ballAngle >= 30) {
            //delta = sqrt(st) * k;
            delta = st * k;
            //speed = 150;
            }
            else if (ballAngle <= -30) {
            //delta = -sqrt(st) * k;
            delta = -st * k;
            // speed = 150;
            }
            else {
            delta = 0;
            // speed = 230;
            }

            moveAngle = ballAngle + delta;

            drive(moveAngle , (int)deltaAngle, 50);
        }
        else{
          drive(goodAngle(lineAngle + 180), deltaAngle, 60);
        }
    }
    else{
        //dribble(30);
        //drive(0, 0, 0, 0);
        int tt = millis();
        while (millis() - tt < 0){
            saveLineDirection();
            getOpenMVDataOmni();
        }
        while (isBall()){
            saveLineDirection();
            getOpenMVDataOmni();

            cam_angle = -omnicam().gates[color].center_angle;
            cam_dist = omnicam().gates[color].distance;
            lineAngle = getLineAngle_Delayed();
            if (lineAngle == 360) {
                int delta_angle = constrain(cam_angle * 0.5, -20, 20);
                drive(cam_angle, delta_angle, 60);
                Serial.println(cam_dist);
                if (abs(cam_angle) < 10 && cam_dist < 35){
                    dribble(0);
                    kick();
                }
            }
            else{
                drive(goodAngle(lineAngle + 180), 0, 60);
            }
        }
    }
  }
}



void playForwardDribble(int color){
  while (true){
    getOpenMVDataOmni();
    saveLineDirection();
    return_kick();
    // if (omnicam().gates[1].width == 0){
    //   dribble(20);
    // }
    // else{
    //   dribble(0);
    // }

    int cam_angle = -omnicam().gates[color].center_angle;
    int cam_dist = omnicam().gates[color].distance;
    
    int st = getStrength();
    int ballAngle = getBallAngle();
    int lineAngle = getLineAngle_Delayed();

    // if (abs(mpuGetDegree()) > 80){
    //     drive(0, -mpuGetDegree() * 0.3, 0);
    // }

    if (!isBall()){
        dribble((abs(ballAngle) < 40) ? 50 : 0);
        int deltaAngle = ballAngle * 0.6;
        if (lineAngle == 360){
            // moveAngle = ballAngle + (ballAngle > 0) ? 15 : -15;
            // if (abs(ballAngle) < 25){
            //     moveAngle = ballAngle;
            // }
            // double k = 0.5;

            // int delta = 0;
            // // if (abs(angle) <= 45){
            // //   k = 3;
            // // }
            // if (ballAngle >= 30) {
            // //delta = sqrt(st) * k;
            // delta = st * k;
            // //speed = 150;
            // }
            // else if (ballAngle <= -30) {
            // //delta = -sqrt(st) * k;
            // delta = -st * k;
            // // speed = 150;
            // }
            // else {
            // delta = 0;
            // // speed = 230;
            // }

            // moveAngle = ballAngle + delta;

            drive(moveAngle , (int)deltaAngle, 60);
        }
        else{
          drive(goodAngle(lineAngle + 180), deltaAngle, 80);
        }
    }
    else{
        dribble(50);
        drive(0, 0, 0, 0);
        int tt = millis();
        while (millis() - tt < 500){
            getOpenMVDataOmni();
            saveLineDirection();
            return_kick();
        }
        while (isBall()){
            getOpenMVDataOmni();
            saveLineDirection();
            return_kick();

            cam_angle = -omnicam().gates[color].center_angle;
            cam_dist = omnicam().gates[color].distance;
            lineAngle = getLineAngle_Delayed();
            dribble((abs(cam_angle) > 10 || cam_dist > 50) ? 50 : 0);
            
            if (lineAngle == 360) {
                int delta_angle = constrain(cam_angle * 0.5, -20, 20);
                drive(cam_angle, delta_angle, 80);
                // Serial.println(cam_dist);
                if (abs(cam_angle) < 10 && cam_dist < 35){
                    dribble(0);
                    kick();
                }
            }
            else{
                drive(goodAngle(lineAngle + 180), 0, 80);
            }
        }
    }
  }
}

void make_pause(int ms){
    int tt = millis();
    while (millis() - tt < ms){
        getBallAngle();
        getOpenMVDataOmni();
        saveLineDirection();
        return_kick();
        // Debug.SendInfo();
    }
}

void playForwardDribble2(int color){
  while (true){
    fwDribbleBegin:
    getOpenMVDataOmni();
    saveLineDirection();
    return_kick();

    // if (omnicam().gates[1].width == 0){
    //   dribble(20);
    // }
    // else{
    //   dribble(0);
    // }

    int cam_angle = -omnicam().gates[color].center_angle;
    int cam_dist = omnicam().gates[color].distance;
    
    int st = getStrength();
    int ballAngle = getBallAngle();
    int lineAngle = getLineAngle_Delayed();

    // if (abs(mpuGetDegree()) > 80){
    //     drive(0, -mpuGetDegree() * 0.3, 0);
    // }

    if (!isBall()){
        dribble((abs(ballAngle) < 40) ? 40 : 0);
        int deltaAngle = ballAngle * 0.6;
        if (lineAngle == 360){
            // moveAngle = ballAngle + (ballAngle > 0) ? 15 : -15;
            // if (abs(ballAngle) < 25){
            //     moveAngle = ballAngle;
            // }
            // double k = 0.5;

            // int delta = 0;
            // // if (abs(angle) <= 45){
            // //   k = 3;
            // // }
            // if (ballAngle >= 30) {
            // //delta = sqrt(st) * k;
            // delta = st * k;
            // //speed = 150;
            // }
            // else if (ballAngle <= -30) {
            // //delta = -sqrt(st) * k;
            // delta = -st * k;
            // // speed = 150;
            // }
            // else {
            // delta = 0;
            // // speed = 230;
            // }

            moveAngle = ballAngle * 0.5; // + delta;

            drive(moveAngle , (int)(deltaAngle * 0.5), 60);
        }
        else{
          drive(goodAngle(lineAngle + 180), (int)(deltaAngle * 0.5), 80);
        }
    }
    else{
        dribble(50);
        drive(0, 0, 0, 0);
        int tt = millis();
        make_pause(500);
        while (isBall()){
            getOpenMVDataOmni();
            saveLineDirection();
            return_kick();
            getBallAngle();

            cam_angle = -omnicam().gates[color].center_angle;
            cam_dist = omnicam().gates[color].distance;
            lineAngle = getLineAngle_Delayed();
            dribble((abs(cam_angle) > 10 || cam_dist > 50) ? 50 : 0);
            
            if (lineAngle == 360) {
                while (abs(cam_angle) < 150){
                    getOpenMVDataOmni();
                    saveLineDirection();
                    getBallAngle();
                    return_kick();
                    lineAngle = getLineAngle_Delayed();
                    moveAngle = goodAngle(lineAngle + 180);
                    speed = (lineAngle == 360) ? 0 : 60;
                    cam_angle = -omnicam().gates[color].center_angle;
                    deltaAngle = ((cam_angle > 0) ? -(180 - cam_angle) : -(-180 - cam_angle)) * 0.5;
                    deltaAngle = constrain(deltaAngle, -maxTorqueWithBall, maxTorqueWithBall);
                    drive(moveAngle, (int)(deltaAngle * 0.5), speed);
                    // Serial.println(cam_angle);
                    // Debug.SendInfo();
                }
                
                // return;
                int delta_angle = ((cam_angle > 0) ? -(180 - cam_angle) : -(-180 - cam_angle)) * 0.5;
                delta_angle = constrain(delta_angle, -maxTorqueWithBall, maxTorqueWithBall);
                drive(cam_angle, delta_angle, maxSpeedBackWithBall);

                saveLineDirection();
                lineAngle = getLineAngle_Delayed();
                if (lineAngle != 360){
                    drive(goodAngle(lineAngle + 180), delta_angle, 80);
                    continue;
                }
                // Serial.println(cam_dist);
                if (cam_dist < 90){
                    // return;
                    drive(0, 0, 0, 0);
                    getOpenMVDataOmni();
                    saveLineDirection();
                    return_kick();
                    dribble(80);

                    make_pause(100);

                    cam_angle = -omnicam().gates[color].right_angle;
                    
                    while (abs(cam_angle) < 150 && isBall()){
                        getOpenMVDataOmni();
                        saveLineDirection();
                        getBallAngle();
                        return_kick();
                        cam_angle = -omnicam().gates[color].right_angle;
                        deltaAngle = ((cam_angle > 0) ? -(180 - cam_angle) : -(-180 - cam_angle)) * 0.5;
                        deltaAngle = constrain(deltaAngle, -20, 20);
                        drive(0, (int)(deltaAngle * 0.5), 0);
                        Serial.println(cam_angle);
                        // Debug.SendInfo();
                    }

                    if (!isBall()){
                        goto fwDribbleBegin;
                    }

                    make_pause(200);

                    if (abs(mpuGetDegree()) < 150 || cam_dist < 60)
                        goalRotate(color);
                    else
                        goalDriveBack(color);
                    
                    Serial.println("Zakrut zakonchen");
                    goto fwDribbleBegin;
                }
            }
            else{
                drive(goodAngle(lineAngle + 180), 0, 80);
            }
            // Debug.SendInfo();
        }
    }
    // Debug.SendInfo();
  }
}

void goalRotate(int color){
    int sign = (mpuGetDegree() < 0) ? -1 : 1;
    while (isBall()){
        getOpenMVDataOmni();
        getBallAngle();
        saveLineDirection();
        return_kick();
        int rotateSpeed = abs(omnicam().gates[color].center_angle) > 120 ? 42 : 100; // 42 : 100
        dribble(abs(omnicam().gates[color].center_angle) > 120 ? 110 : 0); // 110
        drive(0, rotateSpeed * sign, 0);
        // if (omnicam().gates[color].center_angle > 0){
        //     drive(0, -40, 0);
        // }
        // else {
        //     drive(0, 40, 0);
        // }
        // Debug.SendInfo();
    }
}

void goalDriveBack(int color){
    dribble(0);
    while (isBall()){
        getOpenMVDataOmni();
        getBallAngle();
        saveLineDirection();
        return_kick();
        driveXY(0, 100, 0);
    }
    
    ballAngle = getBallAngle();
    int sign = (mpuGetDegree() < 0) ? -1 : 1;
    while (abs(ballAngle) < 40 && !isBall()){
        getOpenMVDataOmni();
        getBallAngle();
        saveLineDirection();
        return_kick();
        ballAngle = getBallAngle();
        driveXY(40 * sign, 0, 0);;
    }
}

void killerFeature(int color)
{
    while (true){
        if (stateGame != 1){
            break;
            // playGoalkeeperCamera(1 ^ color);
        }
        if (millis() - kfTimer >= maxKfTime)
        {
            stateGame = 0;
            ballMoveTime = millis();
            break;
        }

        saveLineDirection();

        getOpenMVDataOmni();
        ballAngle = getBallAngle();
        int robotAngle = mpuGetDegree();
        int gateAngle = omnicam().gates[color].center_angle;

        if (gateAngle != 360)
        {
            gateAngle = (int)goodAngle(gateAngle + 180);
            //lastGateAngle = (int)goodAngle(gateAngle + robotAngle);
            //lastGateTime = millis();
        }
        else
        {
            driveXY(0, 0, 20);
            continue;
            gateAngle = (int)goodAngle(180 - robotAngle);
        }

        int cam_height = omnicam().gates[color].height;

        lineAngle = getLineAngle_Delayed();

        deltaAngle = goodAngle(gateAngle + 180) * 0.25f;

        Serial.print("gateAngle: ");
        Serial.println(gateAngle);

        // if (abs(goodAngle(gateAngle - ballAngle)) >= 140){
        //     stateGame = 0;
        //     continue;
        // }

        // if (abs(deltaAngle) <= 10 && cam_height == 0)
        // {
        //     deltaAngle = 50;
        // }

        if (lineAngle != 360)
        {
            drive(goodAngle(lineAngle + 180), (int)deltaAngle, 50);
        }
        else
        {
            if (getStrength() >= 70 && abs(ballAngle) > 165)
            {
                moveAngle = gateAngle;
                drive(moveAngle, (int)deltaAngle, 80);
            }
            else
            {
                moveAngle = ballAngle;
                /*if (ballAngle >= 0 && ballAngle < 150)
                    moveAngle -= (180 - ballAngle) * 0.6f;
                if (ballAngle < 0 && ballAngle > -150)
                    moveAngle -= (-180 - ballAngle) * 0.6f;*/
                if (ballAngle > 0)
                    moveAngle = goodAngle(ballAngle - getStrength() * goRoundBallCoefGk);
                else
                    moveAngle = goodAngle(ballAngle + getStrength() * goRoundBallCoefGk);

                drive(moveAngle, (int)deltaAngle, 50);
            }
        }

        // if (cam_height >= 700)
        // {
        //     stateGame = 0;
        //     continue;
        // }
    }
}

void playGoalkeeperCamera(int color)
{
    while (true){
        // killerFeature(1 ^ color);
        // continue;
        
        saveLineDirection();
        if (getStrength() < 5)
        {
            drive(0, 0, 0, 0);
            continue;
        }

        if (stateGame != 0)
        {
            kfTimer = millis();
            killerFeature(1 ^ color);
            // Serial.println("KILLER!!!");
            continue;
        }

        ballAngle = getBallAngle();
        int robotAngle = mpuGetDegree();

        float lineX, lineY;
        getLineDirection_Delayed(lineX, lineY);

        getOpenMVDataOmni();
        int gateAngle = omnicam().gates[color].center_angle;
        int cam_height = omnicam().gates[color].height;
        if (cam_height > 0)
        {
            gateAngle = (int)goodAngle(gateAngle + 180);
            lastGateAngle = (int)goodAngle(gateAngle + robotAngle);
            lastGateTime = millis();
        }
        else
        {
            driveXY(0, 0, 20);
            continue;
            // gateAngle = lastGateAngle - robotAngle;
            // cam_height = 110;
        }

        lineAngle = getLineAngle_Delayed();

        int ball_strength = getStrength();

        if (abs(ballAngle + robotAngle - lastBallAngle) > ballNoMotionDiap)
        {
            lastBallAngle = ballAngle + robotAngle;
            lastMoveBallStrength = ball_strength;
            ballMoveTime = millis();
            // Serial.print("angle changed: ");
            // Serial.print(lastBallAngle);
            // Serial.print(" -> ");
            // Serial.print(ballAngle);
            // Serial.println();
        }

        if (abs(ball_strength - lastMoveBallStrength) > ballNoMotionStrength)
        {
            lastBallAngle = ballAngle + robotAngle;
            lastMoveBallStrength = ball_strength;
            ballMoveTime = millis();
            // Serial.print("strength changed: ");
            // Serial.print(lastMoveBallStrength);
            // Serial.print(" -> ");
            // Serial.print(ball_strength);
            // Serial.println();
        }

        // Serial.print("ballAngle:   ");
        // Serial.println(ballAngle);
        // Serial.print("gateAngle:   ");
        // Serial.println(gateAngle);
        // Serial.print("cam_height:   ");
        // Serial.println(cam_height);
        // Serial.print("robotAngle:   ");
        // Serial.println(robotAngle);
        // Serial.print("lineAngle:   ");
        // Serial.println(lineAngle);
        // Serial.println(millis() - ballMoveTime);

        if (millis() - ballMoveTime >= 5000)
        {
            stateGame = 1;
            // Serial.println("state 1!!!");
            continue;
        }

        if (lineAngle != 360 && (abs(goodAngle(robotAngle + gateAngle)) <= 135))
        {
            // speedX = (int)(-lineX * 80);
            // speedY = (int)(-lineY * 80);
            deltaAngle = (gateAngle == 360) ? -robotAngle : -(int)goodAngle(180 - gateAngle);
            drive(goodAngle(lineAngle + 180), deltaAngle, 80);
            continue;
        }
        else
        {
            speedX = 0;
            int err = cam_height - 110;
            speedY = (int)(err * ((err > 0) ? gate_kp : gate_kp_neg) + (err - gatePrev) * gate_kd + gateIntegral);
            speedY = (int)constrain(speedY, -limitGateSpeed, 100);
            gatePrev = err;
            gateIntegral += (err * gate_ki);
            gateIntegral = constrain(gateIntegral, -limitGateIntegral, limitGateIntegral);

            if (abs(ballAngle) > 90 && err < 0)
            {
                deltaAngle = (gateAngle == 360) ? -robotAngle : -(int)goodAngle(180 - gateAngle);
                if (ballAngle > 0)
                    moveAngle = goodAngle(ballAngle + getStrength() * goRoundBallCoefGk);
                else
                    moveAngle = goodAngle(ballAngle - getStrength() * goRoundBallCoefGk);
                drive(moveAngle, (int)deltaAngle, (int)(50));
                //Debug.Log("go round");
                continue;
            }
        }

        ballAngle = constrain(ballAngle, -maxGoalkeeperAngle - robotAngle, maxGoalkeeperAngle - robotAngle);
        float ballSpeed = gb_kp * ballAngle + gkBallIntegral + gb_kd * (ballAngle - gkBallPrev);
        // if (ball_strength > prevBallStrength){
        //     ballSpeed += constrain((ball_strength - prevBallStrength) * gk_st_kd, -50, 50);
        // }
        gkBallIntegral += (ballAngle) * gb_ki;
        gkBallPrev = ballAngle;
        speedX += (int)(ballSpeed);

        deltaAngle = -(int)goodAngle(180 - gateAngle);

        prevBallStrength = ball_strength;

        driveXY(speedX, speedY, (int)deltaAngle);
    }
}

void playGoalkeeperSuperlast(int color){
    while (true){
        saveLineDirection();
        getOpenMVDataOmni();
        
        cameraAngle = -omnicam().gates[color].center_angle;
        ballAngle = getBallAngle();
        Serial.println(cameraAngle);
        if (cameraAngle == 360){
            drive(0, 50, 0);
            continue;
        }
        speedX = cameraAngle * 2;
        
        float lineX, lineY;
        getLineDirection_Delayed(lineX, lineY);

        if (lineX == 0 && lineY == 0){
            speedY = -30;
        }
        else{
            driveXY(-lineX * 70, -lineY * 70, ballAngle * 0.4);
            continue;
        }
        driveXY(speedX, speedY, ballAngle * 0.4);
    }
}

/// Нападающий (простой) ----------------------------------------------------------------------------------------------

void drive2ballSimple()
{
    // получаем необходимые переменные
    dribble(0);
    int st = getStrength();
    ballAngle = getBallAngle();
    lineAngle = getLineAngle_Delayed();

    // если поймали мяч - переходим в состояние ожидания 1
    if (isBall())
    {
        stateGame = 1;
        timeBallHeld = millis();
        return;
    }

    // определяем угол движения
    deltaAngle = ballAngle * 0.3;

    // скорость движения к мячу (замедление вблизи)
    // float err = isBallStrength - st;
    //speed = (int)(drive2ballSpeed + (err - fwBallPrev) * fb_kd);
    ///print($"{name}:   err={err}, deltaErr={err - fwBallPrev}, speed = {speed}");
    // fwBallPrev = err;
    speed = drive2ballSpeed; //constrain(speed, -drive2ballSpeed, drive2ballSpeed);

    if (lineAngle == 360) // если нет линии - едем куда собирались
    {
        drive(ballAngle, (int)deltaAngle, speed);
    }
    else // иначе отъезжаем от линии
    {
        drive(goodAngle(lineAngle + 180), lineSpeed);
    }
}

void move2gateSimple(int color)
{
    while (true){
        // если потеряли мяч - переход в состояние 0 (drive2ball)
        if (!isBall())
        {
            stateGame = 0;
            return;
        }

        dribble(dribblerSpeed); // дриблим мяч

        // нужные переменные
        getOpenMVDataOmni();
        cameraAngle = -omnicam().gates[color].center_angle; // getCamData(color);
        //lineAngle = getLineAngle_Delayed();
        float lineX, lineY;
        getLineDirection_Delayed(lineX, lineY);
        lineX = 0; lineY = 0;
        int cam_height = omnicam().gates[color].height; // getCamHeight();
        int robotAngle = mpuGetDegree();

        if (lineX == 0 && lineY == 0) // если нет линии
        {
            // if (cameraAngle == 360) // если не видим ворота
            // {
            //     if (abs(robotAngle) < 10) // при этом смотрим вперёд - мы в переднем углу, едем назад
            //     {
            //         drive(180, -robotAngle, (int)(move2gateSpeed * 0.5f));
            //     }
            //     else // иначе поворачиваемся вперёд в оптимальном направлении
            //     {
            //         if (robotAngle < 0)
            //             drive(0, maxTorqueWithBall, 0);
            //         else
            //             drive(0, -maxTorqueWithBall, 0);
            //     }
            // }
            // else // едем к воротам
            // {
            // объезжаем потенциальное препятствие

            moveAngle = cameraAngle;
            float speedX = move2gateSpeed * sin(moveAngle * DEG_TO_RAD);
            float speedY = move2gateSpeed * cos(moveAngle * DEG_TO_RAD);
            deltaAngle = cameraAngle * 0.7;

            // float offset = constrain(cameraAngle, -15, 15);

            // float k = goRoundObstacleCoef;
            // if (cam_height >= 60) // если ворота ближе - то объезжаем препятствие быстрее
            //     k *= 2;
            // speedX += offset * k;

            driveXY((int)speedX, (int)speedY, (int)(deltaAngle));

            //drive(cameraAngle, (int)cameraAngle, move2gateSpeed);
            // }
        }
        else // видим линию - уезжаем от неё
        {
            speedX = (int)(-lineSpeed * lineX);
            speedY = (int)(-lineSpeed * lineY);
            driveXY(speedX, speedY, 0);
        }

        Serial.print("camera with color ");
        Serial.print(color);
        Serial.print(":");
        Serial.println(cameraAngle);

        // забиваем гол
        if (cam_height >= 100 && cameraAngle != 360 && abs(cameraAngle) < 20)
        {
            dribble(0);
            kick();
            return;
        }
    }
}

void playForwardSimple(int color)
{
    while (true) {
        saveLineDirection();
        if (getStrength() < 5)
        {
            drive(0, 0, 0, 0);
            continue;
        }
        switch (stateGame)
        {
            case 0: // 0 - едем к мячу
                drive2ballSimple();
                break;
            case 1: // 1 - ждём, пока мяч захватится
                dribble(dribblerSpeed);
                drive(0, 0, 0, 0);
                saveLineDirection();
                lineAngle = getLineAngle_Delayed();
                if (lineAngle != 360)
                {
                    if (lineAngle == 0)
                    {
                        drive(180, lineSpeed);
                    }
                    else
                        drive(goodAngle(lineAngle + 180), 200);
                }
                if (millis() - timeBallHeld >= gateMovePause)
                {
                    stateGame = 2;
                }
                break;
            case 2: // 2 - едем к воротам соперника
                move2gateSimple(1 ^ color);
                break;
        }
    }
}

void set_kp(int value){
    g_kp = value;
}

/// Вратарь (следование по линии) -------------------------------------------------------------------------------------

int killer_timer = millis();
int prevBallAngle = 0;

void playGoalkeeperFollowLine()
{
    while (true){
        //getOpenMVDataOmni();
        saveLineDirection();
        if (getStrength() < 5)
        {
            drive(0, 0, 0, 0);
            continue;
        }
        ballAngle = getBallAngle();

        int robotAngle = mpuGetDegree();
        int rotateSpeed = 0;
        speedX = speed;
        speedY = 0;

        lineAngle = getLineAngle_Delayed();

        // while (abs(cam_angle) > 20){
        //     getOpenMVDataOmni();
        //     saveLineDirection();
        //     lineAngle = getLineAngle_Delayed();
        //     if ( omnicam().gates[1].center_angle == 360)
        //         cam_angle = -robotAngle;
        //     else
        //         cam_angle = omnicam().gates[1].center_angle;
        //     if (abs(cam_angle) <= 20)
        //         break;
        //     cam_dist = omnicam().gates[1].distance;

        //     moveAngle = (lineAngle == 360) ? 0 : -lineAngle;
        //     speed = (lineAngle == 360) ? 0 : 40;
        //     drive(moveAngle, -cam_angle * 0.3, speed);
        // }

        // while (abs(cam_angle) <= 20){
        saveLineDirection();
        lineAngle = getLineAngle_Delayed();
        // if (lineAngle != 360){
        //     setLastLineDirection(lineAngle);
        // }

        if (prevBallAngle != ballAngle){
            killer_timer = millis();
        }
        // if (abs(ballAngle - prevBallAngle) == 0 && millis() - killer_timer > 5000){
        //     killer_timer = millis();
        //     while (millis() - killer_timer < 1000){
        //         drive(0, 0, 80);
        //         saveLineDirection();
        //         getOpenMVDataOmni();
        //     }
        //     while (getStrength() < 160){
        //         drive(0, 0, 80);
        //         saveLineDirection();
        //         getOpenMVDataOmni();
        //         lineAngle = getLineAngle_Delayed();
        //         if (lineAngle != 360){
        //             drive(goodAngle(lineAngle + 180), 0, 80);
        //             // break;
        //         }
        //     }
        // }
        // prevBallAngle = ballAngle;

        int v1 = 0;
        int v2 = 0;
        int v3 = 0;
        int v4 = 0;

        float leftSpeed = 0;
        float rightSpeed = 0;
        int leftk = 0, rightk = 0;
        int leftSign = -1, rightSign = -1;

        // Serial.print("millis: ");
        // Serial.println(millis());
        for (int i = 0; i < 8; ++i)
        {
            float priority = (millis() - getLineTime(i));
            // Serial.print("left sensor ");
            // Serial.print(i);
            // Serial.print(", line time ");
            Serial.print(getLineTime(i) > 0);
            Serial.print(" ");
            // Serial.print(",  priority = ");
            // Serial.print(priority);
            if (priority < considerLineTime){
                // Serial.print(", start bottom part");
                leftSign = 1;
            }
            else
            {
                leftSpeed += leftSign * lineSensorPriority[i];
                // Serial.print(", += ");
                // Serial.print(leftSign * lineSensorPriority[i]);
                leftk += 1;
            }
            // Serial.println();
        }
        // Serial.println();
        for (int i = 15; i >= 8; --i)
        {
            float priority = (millis() - getLineTime(i));
            // Serial.print("left sensor ");
            // Serial.print(i);
            // Serial.print(", line time ");
            Serial.print(getLineTime(i) > 0);
            Serial.print(" ");
            // Serial.print(",  priority = ");
            // Serial.print(priority);
            // Serial.print(",  need = ");
            // Serial.print(considerLineTime * 15);
            if (priority < considerLineTime){
                // Serial.print(", start bottom part");
                rightSign = 1;
            }
            else
            {
                rightSpeed += rightSign * lineSensorPriority[i];
                // Serial.print(", += ");
                // Serial.print(rightSign * lineSensorPriority[i]);
                rightk += 1;
            }
            // Serial.println();
        }
        Serial.println();

        if (leftk > 0)
            leftSpeed /= leftk;
        if (rightk > 0)
            rightSpeed /= rightk;
        
        // Serial.print("left: ");
        // Serial.print(leftSpeed);
        // Serial.print(",   right: ");
        // Serial.print(rightSpeed);
        // Serial.println();

        // Serial.print("previous    left: ");
        // Serial.print(leftPrev);
        // Serial.print(",   right: ");
        // Serial.print(rightPrev);
        // Serial.println();
        
        // Serial.print("leftk: ");
        // Serial.print(leftk);
        
        // Serial.print(",    rightk: ");
        // Serial.println(rightk);

        if (leftk == activeSensors || rightk == activeSensors)
        {
            leftIntegral = 0;
            rightIntegral = 0;
            leftPrev = 0;
            rightPrev = 0;
        }

        if (leftk == activeSensors && rightk == activeSensors)
        {
            //Debug.Log("NO LINE:");
            leftSpeed = 0;
            rightSpeed = 0;
            if (abs(ballAngle) < 90)
            {
                rotateSpeed = -robotAngle * 0.3; // (int)goodAngle((180 + lastLineAngle - robotAngle));
                //drive((int)goodAngle(lastLineAngle - robotAngle), 0, (int)(horizCoef * 90));
                //return;
                //speedY = -goalkeeperVertSpeed;
                drive(180, rotateSpeed, (int)(40));
                Serial.println("back");
                continue;
            }
            else
            {
                rotateSpeed = -robotAngle * 0.3;
                if (ballAngle > 0)
                    moveAngle = goodAngle(ballAngle + getStrength() * goRoundBallCoefGk);
                else
                    moveAngle = goodAngle(ballAngle - getStrength() * goRoundBallCoefGk);
                drive(moveAngle, rotateSpeed, (int)(40));
                Serial.println("move round");
                continue;
            }
        }
        /*else if (leftk == 8)
        {
            leftSpeed = -100;
            leftIntegral = 0;
            leftPrev = 0;
        }
        else if (rightk == 8)
        {
            rightSpeed = -100;
            rightIntegral = 0;
            rightPrev = 0;
        }*/

        float lineX, lineY;
        getLineDirection_Delayed(lineX, lineY);

        float speedK = 1;
        if (leftSpeed + rightSpeed != 0)
            speedK = constrain(1 - abs(leftSpeed - rightSpeed) * rotateSlowingCoef, 0, 1);
        // Serial.print("speedK: ");
        // Serial.println(speedK);
        /*if (speedK <= 0)
        {
            speedX += (int)(lineX * 100);
            speedY += (int)(lineY * 100);
        }*/

        //ballAngle = constrain(ballAngle, -maxGoalkeeperAngle - robotAngle, maxGoalkeeperAngle - robotAngle);
        ballAngle = constrain(ballAngle, -90, 90);
        float ballSpeed = gb_kp * ballAngle + gkBallIntegral + gb_kd * (ballAngle - gkBallPrev);
        gkBallIntegral += (ballAngle) * gb_ki;
        gkBallPrev = ballAngle;

        if (leftSpeed > rightSpeed && robotAngle >= maxGoalkeeperAngle)
        {
            //Debug.Log("go round");
            if (ballAngle < -gkReactOnBallDiap)
                speedX = (int)(ballSpeed * speedK);
            else
            {
                drive(-90, 10);
                continue;
            }
        }
        else if (leftSpeed < rightSpeed && robotAngle <= -maxGoalkeeperAngle)
        {
            if (ballAngle > gkReactOnBallDiap)
                speedX = (int)(ballSpeed * speedK);
            else
            {
                drive(90, 10);
                continue;
            }
        }
        else
        {
            if (ballAngle > gkReactOnBallDiap)
                speedX = (int)(ballSpeed * speedK);
            if (ballAngle < -gkReactOnBallDiap)
                speedX = (int)(ballSpeed * speedK);
        }
        //speedX = (int)(ballSpeed * speedK);

        v1 = (int)(g_kp * leftSpeed + leftIntegral + g_kd * (leftSpeed - leftPrev));
        v2 = v1;
        v3 = (int)(g_kp * rightSpeed + rightIntegral + g_kd * (rightSpeed - rightPrev));
        v4 = v3;
        // Serial.print("motors: ");
        // Serial.print(v1);
        // Serial.print(" ");
        // Serial.print(v2);
        // Serial.print(" ");
        // Serial.print(v3);
        // Serial.print(" ");
        // Serial.println(v4);

        leftIntegral += (leftSpeed) * g_ki;
        leftPrev = leftSpeed;
        rightIntegral += (rightSpeed) * g_ki;
        rightPrev = rightSpeed;

        v1 += (int)((speedY + speedX)) + rotateSpeed;
        v2 += (int)((speedY - speedX)) + rotateSpeed;
        v3 += (int)((speedY + speedX)) - rotateSpeed;
        v4 += (int)((speedY - speedX)) - rotateSpeed;

        drive(v1, v2, v3, v4);
    }

}

void playGoalkeeperGoyda(int color)
{
    while (true){
        getOpenMVDataOmni();
        saveLineDirection();
        if (getStrength() < 5)
        {
            drive(0, 0, 0, 0);
            continue;
        }
        ballAngle = getBallAngle();

        int robotAngle = mpuGetDegree();
        int rotateSpeed = 0;
        speedX = speed;
        speedY = 0;

        getOpenMVDataOmni();
        int gateAngle = omnicam().gates[color].center_angle;

        int cam_angle = omnicam().gates[color].center_angle;
        bool followLine = (gateAngle != 360);
        if (gateAngle != 360)
        {
            gateAngle = (int)goodAngle(gateAngle + 180);
            lastGateAngle = (int)goodAngle(gateAngle + robotAngle);
        }
        else
        {
            lastGateAngle = 180;
        }
        
        getOpenMVDataOmni();
        saveLineDirection();
        lineAngle = getLineAngle_Delayed();
        if (lineAngle != 360 && followLine)
        {
            lastLineAngle = lineAngle + robotAngle;
            lastLineTime = millis();
        }

        if (millis() - lastLineTime >= lastlineReset)
        {
            lastLineAngle = 180;
        }

        if (stateGame == 1)
        {
            speed = (lineAngle == 360) ? 200 : 50;
            drive(lastLineAngle - robotAngle, -robotAngle, speed);
            if (abs(robotAngle) < 45)
            {
                stateGame = 0;
            }
            return;
        }

        int v1 = 0;
        int v2 = 0;
        int v3 = 0;
        int v4 = 0;

        float leftSpeed = 0;
        float rightSpeed = 0;
        int leftk = 0, rightk = 0;
        int leftSign = -1, rightSign = -1;

        // Serial.print("millis: ");
        // Serial.println(millis());
        for (int i = 0; i < 8; ++i)
        {
            float priority = (millis() - getLineTime(i));
            // Serial.print("left sensor ");
            // Serial.print(i);
            // Serial.print(", line time ");
            // Serial.print(getLineTime(i) > 0);
            // Serial.print(" ");
            // Serial.print(",  priority = ");
            // Serial.print(priority);
            if (priority < considerLineTime){
                // Serial.print(", start bottom part");
                leftSign = 1;
            }
            else
            {
                leftSpeed += leftSign * lineSensorPriority[i];
                // Serial.print(", += ");
                // Serial.print(leftSign * lineSensorPriority[i]);
                leftk += 1;
            }
            // Serial.println();
        }
        // Serial.println();
        for (int i = 15; i >= 8; --i)
        {
            float priority = (millis() - getLineTime(i));
            // Serial.print("left sensor ");
            // Serial.print(i);
            // Serial.print(", line time ");
            // Serial.print(getLineTime(i) > 0);
            // Serial.print(" ");
            // Serial.print(",  priority = ");
            // Serial.print(priority);
            // Serial.print(",  need = ");
            // Serial.print(considerLineTime * 15);
            if (priority < considerLineTime){
                // Serial.print(", start bottom part");
                rightSign = 1;
            }
            else
            {
                rightSpeed += rightSign * lineSensorPriority[i];
                // Serial.print(", += ");
                // Serial.print(rightSign * lineSensorPriority[i]);
                rightk += 1;
            }
            // Serial.println();
        }
        // Serial.println();

        if (leftk > 0)
            leftSpeed /= leftk;
        if (rightk > 0)
            rightSpeed /= rightk;

        float lineX, lineY;
        getLineDirection_Delayed(lineX, lineY);

        if (gateAngle == 360)
        {
            //moveAngle = (lineAngle == 360) ? 0 : goodAngle(lineAngle + 180);
            //speed = (lineAngle == 360) ? 0 : lineSpeed;

            speedX = (lineAngle == 360) ? (int)(ballAngle) : (int)(-lineX * lineSpeed);
            speedY = (lineAngle == 360) ? -100 : (int)(-lineY * lineSpeed);
            rotateSpeed = -(int)goodAngle(180 - (lastGateAngle - robotAngle));
            if (lastGateAngle == 360)
                rotateSpeed = -robotAngle;

            leftSpeed = 0;
            rightSpeed = 0;
            leftPrev = 0;
            rightPrev = 0;
            leftIntegral = 0;
            rightIntegral = 0;
            //drive(moveAngle, 50, speed);
            //return;
        }

        else if (leftk == 8 && rightk == 8)
        {
            //GetComponent<SpriteRenderer>().color = Color.blue;
            //Debug.Log("NO LINE:");
            leftSpeed = 0;
            rightSpeed = 0;


            if (abs(ballAngle + robotAngle) < 90)
            {
                //rotateSpeed = -robotAngle; // (gateAngle == 360) ? -robotAngle : (int)goodAngle(gateAngle);
                rotateSpeed = -(int)goodAngle(lastGateAngle - robotAngle);
                if (lastGateAngle == 360)
                    rotateSpeed = -robotAngle;
                moveAngle = goodAngle(lastLineAngle - robotAngle); // (gateAngle == 360) ? lastLineAngle - robotAngle : gateAngle;
                //drive((int)goodAngle(lastLineAngle - robotAngle), 0, (int)(horizCoef * 90));
                //speedX = (ballAngle > 0) ? 50 : -50;
                //driveXY(speedX, -200, rotateSpeed);
                
                if (lineAngle != 360)
                {
                    drive(goodAngle(lineAngle + 180), rotateSpeed, lineSpeed);
                    return;
                }
                speedX = (int)(sin(moveAngle * DEG_TO_RAD) * 200);
                speedY = (int)(cos(moveAngle * DEG_TO_RAD) * 200);
                speedX += (int)ballAngle;
                driveXY(speedX, speedY, rotateSpeed);
                //drive(moveAngle, rotateSpeed, 200);
                //Debug.Log("back");
                return;
            }
            else
            {
                rotateSpeed = -robotAngle; // (gateAngle == 360) ? -robotAngle : gateAngle;
                if (ballAngle > 0)
                    moveAngle = goodAngle(ballAngle + getStrength() * goRoundBallCoefGk);
                else
                    moveAngle = goodAngle(ballAngle - getStrength() * goRoundBallCoefGk);

                if (lineAngle != 360)
                {
                    drive(goodAngle(lineAngle + 180), rotateSpeed, lineSpeed);
                    return;
                }
                /*speedX = (int)(Mathf.Sin(moveAngle * Mathf.Deg2Rad) * 200);
                speedY = (int)(Mathf.Cos(moveAngle * Mathf.Deg2Rad) * 200);
                speedX += (int)ballAngle;
                driveXY(speedX, speedY, rotateSpeed);*/
                drive(moveAngle, rotateSpeed, (int)(200));
                //Debug.Log("go round");
                return;
            }
        }
        else
        {
            //GetComponent<SpriteRenderer>().color = Color.white;
        }
        
        // Serial.print("left: ");
        // Serial.print(leftSpeed);
        // Serial.print(",   right: ");
        // Serial.print(rightSpeed);
        // Serial.println();

        // Serial.print("previous    left: ");
        // Serial.print(leftPrev);
        // Serial.print(",   right: ");
        // Serial.print(rightPrev);
        // Serial.println();
        
        // Serial.print("leftk: ");
        // Serial.print(leftk);
        
        // Serial.print(",    rightk: ");
        // Serial.println(rightk);

        if (leftk == activeSensors || rightk == activeSensors)
        {
            leftIntegral = 0;
            rightIntegral = 0;
            leftPrev = 0;
            rightPrev = 0;
        }

        // if (leftk == activeSensors && rightk == activeSensors)
        // {
        //     //Debug.Log("NO LINE:");
        //     leftSpeed = 0;
        //     rightSpeed = 0;
        //     if (abs(ballAngle) < 90)
        //     {
        //         rotateSpeed = gateAngle * 0.3; // (int)goodAngle((180 + lastLineAngle - robotAngle));
        //         //drive((int)goodAngle(lastLineAngle - robotAngle), 0, (int)(horizCoef * 90));
        //         //return;
        //         //speedY = -goalkeeperVertSpeed;
        //         moveAngle = (lineAngle == 360 || followLine) ? 180 : goodAngle(lineAngle + 180);
        //         drive(moveAngle, rotateSpeed, (int)(40));
        //         Serial.println("back");
        //         continue;
        //     }
        //     else
        //     {
        //         rotateSpeed = gateAngle * 0.3;
        //         if (lineAngle == 360 || followLine){
        //             if (ballAngle > 0)
        //                 moveAngle = goodAngle(ballAngle + getStrength() * goRoundBallCoefGk);
        //             else
        //                 moveAngle = goodAngle(ballAngle - getStrength() * goRoundBallCoefGk);
        //             drive(moveAngle, rotateSpeed, (int)(40));
        //         }
        //         else{
        //             drive(goodAngle(lineAngle + 180), rotateSpeed, (int)(40));
        //         }
        //         Serial.println("move round");
        //         continue;
        //     }
        // }
        /*else if (leftk == 8)
        {
            leftSpeed = -100;
            leftIntegral = 0;
            leftPrev = 0;
        }
        else if (rightk == 8)
        {
            rightSpeed = -100;
            rightIntegral = 0;
            rightPrev = 0;
        }*/

        if (followLine && abs(robotAngle) > 110)
        {
            stateGame = 1;
            return;
        }

        float speedK = 1;
        if (followLine && leftSpeed + rightSpeed != 0)
            speedK = constrain(1 - abs(leftSpeed - rightSpeed) * rotateSlowingCoef, 0, 1);
        // Serial.print("speedK: ");
        // Serial.println(speedK);
        /*if (speedK <= 0)
        {
            speedX += (int)(lineX * 100);
            speedY += (int)(lineY * 100);
        }*/

        //ballAngle = constrain(ballAngle, -maxGoalkeeperAngle - robotAngle, maxGoalkeeperAngle - robotAngle);
        ballAngle = constrain(ballAngle, -90, 90);
        float ballSpeed = gb_kp * ballAngle + gkBallIntegral + gb_kd * (ballAngle - gkBallPrev);
        gkBallIntegral += (ballAngle) * gb_ki;
        gkBallPrev = ballAngle;

        if (followLine)
        {
            if (leftSpeed > rightSpeed && robotAngle >= maxGoalkeeperAngle)
            {
                //Debug.Log("go round");
                if (ballAngle < -gkReactOnBallDiap)
                    speedX = (int)(ballSpeed * speedK);
                else
                {
                    drive(-90, 10);
                    continue;
                }
            }
            else if (leftSpeed < rightSpeed && robotAngle <= -maxGoalkeeperAngle)
            {
                if (ballAngle > gkReactOnBallDiap)
                    speedX = (int)(ballSpeed * speedK);
                else
                {
                    drive(90, 10);
                    continue;
                }
            }
            else
            {
                if (ballAngle > gkReactOnBallDiap)
                    speedX = (int)(ballSpeed * speedK);
                if (ballAngle < -gkReactOnBallDiap)
                    speedX = (int)(ballSpeed * speedK);
            }
        }
        else{
            speedX += (int)(ballSpeed * speedK);
        }
        //speedX = (int)(ballSpeed * speedK);
        
        if (followLine){
            v1 = (int)(g_kp * leftSpeed + leftIntegral + g_kd * (leftSpeed - leftPrev));
            v2 = v1;
            v3 = (int)(g_kp * rightSpeed + rightIntegral + g_kd * (rightSpeed - rightPrev));
            v4 = v3;
            // Serial.print("motors: ");
            // Serial.print(v1);
            // Serial.print(" ");
            // Serial.print(v2);
            // Serial.print(" ");
            // Serial.print(v3);
            // Serial.print(" ");
            // Serial.println(v4);

            leftIntegral += (leftSpeed) * g_ki;
            leftPrev = leftSpeed;
            rightIntegral += (rightSpeed) * g_ki;
            rightPrev = rightSpeed;
        }

        v1 += (int)((speedY + speedX)) + rotateSpeed;
        v2 += (int)((speedY - speedX)) + rotateSpeed;
        v3 += (int)((speedY + speedX)) - rotateSpeed;
        v4 += (int)((speedY - speedX)) - rotateSpeed;
        
        drive(v1, v2, v3, v4);
    }
}

void dribblerGoalGoyda(int color){

    // drive(0, 0, 0, 0);
    dribble(0);

    drive(0, getBallAngle() * 0.4, 0);
    int cam_angle, tt;
    while (isBall()){
        getOpenMVDataOmni();
        dribble(50);

        tt = millis();
        while (millis() - tt < 500){
            getOpenMVDataOmni();
        }

        cam_angle = -omnicam().gates[color].center_angle;
        
        while (abs(cam_angle) < 160){
            getOpenMVDataOmni();
            cam_angle = -omnicam().gates[color].center_angle;
            deltaAngle = ((cam_angle > 0) ? -(180 - cam_angle) : -(-180 - cam_angle)) * 0.5;
            deltaAngle = constrain(deltaAngle, -20, 20);
            drive(0, deltaAngle, 0);
            Serial.println(cam_angle);
        }

        tt = millis();
        while (millis() - tt < 500){
            getOpenMVDataOmni();
        }

        while (isBall()){
            getOpenMVDataOmni();
            dribble(60);
            if (mpuGetDegree() > 0){
                drive(0, -50, 0);
            }
            else {
                drive(0, 50, 0);
            }
        }
    }
}

void playGoalkeeperOld()
{
    saveLineDirection();
    if (getStrength() < 5)
    {
        drive(0, 0, 0, 0);
        return;
    }
    //lineAngle = getLineAngle_Delayed();
    float lineX, lineY;
    getLineDirection_Delayed(lineX, lineY);
    //lineAngle = getLineAngle_Avg();
    ballAngle = getBallAngle();

    deltaAngle = -mpuGetDegree();

    mainSpeedY += accelerationY;
    if (mainSpeedY < minSpeedY)
    {
        mainSpeedY = minSpeedY;
    }

    speedY = (int)mainSpeedY;
    if (ballAngle > 120)
    {
        speedX = -30;
        stuckTime = millis();
    }
    if (ballAngle < -120)
    {
        speedX = 30;
        stuckTime = millis();
    }
    else
    {
        if (abs(ballAngle + deltaAngle) > 90)
        {
            //stuckTime = millis();
            speedX = (int)(sin((180 - (ballAngle + deltaAngle)) * DEG_TO_RAD) * horizCoef);
        }
        else
        {
            //stuckTime = millis();
            speedX = (int)(sin((ballAngle + deltaAngle) * DEG_TO_RAD) * horizCoef);
        }
    }

    if (abs(ballAngle) < 10)
        stuckTime = millis();

    if (ballSide * ballAngle < 0)
    {
        ballSide = -ballSide;
        stuckTime = millis();
    }
    if (millis() - stuckTime > timeToPush)
    {
        mainSpeedY = 50;
        stuckTime = millis();
    }

    if (lineX == 0 && lineY == 0)
    {

    }
    else
    {
        //deltaAngle = goodAngle(180 + Mathf.Atan2(lineX, lineY) * Mathf.Rad2Deg);
        //speedX = -(int)(lineX * goalkeeperVertSpeed);
        //speedY = -(int)(lineY * goalkeeperVertSpeed);
        float x, y;
        projectSpeedOnLineXY(speedX, speedY, lineX, lineY, x, y);
        speedX = (int)(x - lineX * goalkeeperLineSpeed);
        speedY = (int)(y - lineY * lineSpeed);
    }

    driveXY(speedX, speedY, (int)deltaAngle);
}

// void drive2ballOmni()
// {
//     // получаем необходимые переменные
//     dribble(0);
//     int st = getStrength();
//     ballAngle = getBallAngle();
//     lineAngle = getLineAngle_Delayed();
//     //OmniCamBlobInfo_t obstacle = getClosestObject(2);

//     // если поймали мяч - переходим в состояние ожидания 1
//     if (isBall())
//     {
//         stateGame = 1;
//         timeBallHeld = millis();
//         return;
//     }

//     // определяем угол движения
//     deltaAngle = ballAngle * 0.3;

//     // скорость движения к мячу (замедление вблизи)
//     // float err = isBallStrength - st;
//     // speed = (int)(drive2ballSpeed + (err - fwBallPrev) * fb_kd);
//     // //print($"{name}:   err={err}, deltaErr={err - fwBallPrev}, speed = {speed}");
//     // fwBallPrev = err;
//     // speed = constrain(speed, -drive2ballSpeed, drive2ballSpeed);
//     speed = 40;

//     if (lineAngle == 360) // если нет линии - едем куда собирались
//     {
//         // if (st < isBallStrength * 0.6f) // если мяч достаточно далеко, едем к нему с объездом препятствий
//         // {
//         //     speed = drive2ballSpeed;
//         //     goOverObstacleOmni(speed, ballAngle, obstacle, criticalObstacleDistance * 0.5f, false);
//         // }
//         // else // если мяч близко, едем к нему прямо
//         // {
//         //     drive(ballAngle, (int)deltaAngle, speed);
//         // }
//         drive(ballAngle, (int)deltaAngle, speed);
//     }
//     else // иначе отъезжаем от линии
//     {
//         drive(goodAngle(lineAngle + 180), 60);
//     }
//     drive(ballAngle, (int)deltaAngle, speed);
//     ballAngle = getBallAngle();
//     st = getStrength();
// }

// void move2gateOmni(int color)
// {
//     // если потеряли мяч - переход в состояние 0 (drive2ball)
//     if (!isBall())
//     {
//         stateGame = 0;
//         return;
//     }

//     dribble(dribblerSpeed); // дриблим мяч

//     // нужные переменные
//     deltaAngle = 0;
//     // OmniCamBlobInfo_t gate = getLargestObject(color);
//     // float cam_distance = 1000;
//     // if (gate != nullptr)
//     // {
//     //     cameraAngle = gate.closest;
//     //     cam_distance = gate.distance;
//     // }
//     float lineX, lineY;

//     saveLineDirection();
//     getLineDirection_Delayed(lineX, lineY);
//     int robotAngle = mpuGetDegree();

//     // OmniCamBlobInfo_t obstacle = getClosestObject(2);
//     // /*float obstacle_distance = 1000;
//     // float obstacle_angle = 360;
//     // if (obstacle != null)
//     // {
//     //     obstacle_angle = obstacle.center;
//     //     obstacle_distance = obstacle.distance;
//     // }*/

//     // // забиваем гол
//     // if (cam_distance <= 6 && gate != null)
//     // {
//     //     stateGame = 3;
//     // }

//     if (lineX == 0 && lineY == 0) // если нет линии
//     {
//         drive(-robotAngle, -robotAngle * 0.3, 20);
//         // if (gate == null) // если не видим ворота
//         // {
//         //     goOverObstacleOmni(maxSpeedBackWithBall, -robotAngle, obstacle, criticalObstacleDistance);
//         //     /*if (Mathf.Abs(robotAngle) < 10) // при этом смотрим вперёд - мы в переднем углу, едем назад
//         //     {
//         //         drive(180, -robotAngle, maxSpeedBackWithBall);
//         //     }
//         //     else // иначе поворачиваемся вперёд в оптимальном направлении
//         //     {
//         //         if (robotAngle < 0)
//         //             drive(0, maxTorqueWithBall, 0);
//         //         else
//         //             drive(0, -maxTorqueWithBall, 0);
//         //     }*/
//         // }
//         // else // едем к воротам
//         // {
//         //     // объезжаем потенциальное препятствие

//         //     goOverObstacleOmni(move2gateSpeed, cameraAngle, obstacle, criticalObstacleDistance);
//         // }
//     }
//     else // видим линию - уезжаем от неё
//     {
//         speedX = (int)(-lineSpeed * lineX);
//         speedY = (int)(-lineSpeed * lineY);
//         driveXY(speedX, speedY, (int)deltaAngle);
//     }
// }

// // функция, чтобы ехать в опр. направлении, объезжая препятствия
// void goOverObstacleOmni(float generalSpeed, float generalAngle, OmniCamBlobInfo_t obstacle, float critDist, bool rotate = true)
// {
//     // определяем параметры препятствия
//     float obstacle_distance = 1000;
//     float obstacle_angle = 360;
//     if (obstacle.width != 0)
//     {
//         obstacle_angle = obstacle.closest;
//         obstacle_distance = obstacle.distance;
//     }

//     // стартовые параметры для drive
//     moveAngle = generalAngle;
//     deltaAngle = generalAngle;
//     if (abs(deltaAngle) > maxTorqueWithBall)
//     {
//         deltaAngle = (deltaAngle > 0) ? maxTorqueWithBall : -maxTorqueWithBall;
//     }

//     // если рядом препятствие
//     if (obstacle.width != 0 && obstacle_distance < critDist)
//     {
//         float co_angle = goodAngle(obstacle_angle - generalAngle); // разность углов ворот и препятствия
//         if (rotate) // если нужно отворачиваться от препятствия, то отворачиваемся
//         {
//             if (co_angle > 0 && co_angle < criticalObstacleAngle)
//             {
//                 deltaAngle = -maxTorqueWithBall;
//             }
//             if (co_angle < 0 && co_angle > -criticalObstacleAngle)
//             {
//                 deltaAngle = maxTorqueWithBall;
//             }
//         }

//         if (abs(co_angle) < criticalObstacleAngle) // если препятствие не сзади
//         {
//             // проецируем скорость поперёк препятствия

//             if (co_angle > 10)
//             {
//                 moveAngle = goodAngle(obstacle_angle - 90);
//             }
//             else if (co_angle < -10)
//             {
//                 moveAngle = goodAngle(obstacle_angle + 90);
//             }
//             else
//             {
//                 moveAngle = goodAngle(obstacle_angle + 120);
//             }
//         }
//     }

//     drive((int)moveAngle, (int)deltaAngle, (int)generalSpeed);
// }

// void playForwardOmni(int color)
// {
//     saveLineDirection();
//     if (getStrength() < 5)
//     {
//         drive(0, 0, 0, 0);
//         return;
//     }
//     switch (stateGame)
//     {
//         case 0: // 0 - едем к мячу
//             drive2ballOmni();
//             break;
//         case 1: // 1 - ждём, пока мяч захватится
//             dribble(dribblerSpeed);
//             drive(0, 0, 0, 0);
//             saveLineDirection();
//             lineAngle = getLineAngle_Delayed();
//             if (lineAngle != 360)
//             {
//                 if (lineAngle == 0)
//                 {
//                     drive(180, lineSpeed);
//                 }
//                 else
//                     drive(goodAngle(lineAngle + 180), 200);
//             }
//             if (millis() - timeBallHeld >= gateMovePause)
//             {
//                 stateGame = 2;
//             }
//             break;
//         case 2: // 2 - едем к воротам соперника
//             move2gateOmni(1 ^ color);
//             break;
//         case 3: // 3 - гол дриблером
//             //dribblerGoalOmni(1 ^ color);
//             break;
//     }
// }

// void dribblerGoalOmni(int color)
// {
//     // если потеряли мяч - переход в состояние 0 (drive2ball)
//     if (!isBall())
//     {
//         stateGame = 0;
//         return;
//     }

//     dribble(dribblerSpeed); // дриблим мяч

//     // нужные переменные
//     deltaAngle = 0;
//     OmniCamBlobInfo_t gate = getLargestObject(color);
//     float cam_distance = 1000;
//     if (gate != nullptr)
//     {
//         cameraAngle = gate.midAngle;
//         cam_distance = gate.distance;
//     }
//     else // если не видим ворот, переходим в состояние 2 (move2gate)
//     {
//         stateGame = 2;
//         return;
//     }
//     float lineX, lineY;
//     getLineDirection_Delayed(lineX, lineY);
//     int robotAngle = mpuGetDegree();

//     // определяем, можно ли сейчас забить гол прямым ударом
//     bool gateInFront = (gate.leftAngle <= -20 && gate.rightAngle >= 20);
//     if (gate.width < 60)
//     {
//         gateInFront = abs(cameraAngle) < 20;
//     }

//     if (cam_distance <= 6)
//     {
//         if (!gateInFront)
//         {
//             deltaAngle = (cameraAngle > 0) ? maxTorqueWithBall : -maxTorqueWithBall;
//         }
//         else
//         {
//             dribble(0);
//             kick();
//         }
//     }
//     //если ворота далеко, переходим в состояние 2 (move2gate)
//     else
//     {
//         stateGame = 2;
//         return;
//     }

//     if (!(lineX == 0 && lineY == 0)) // видим линию - уезжаем от неё
//     {
//         speedX = (int)(-lineSpeed * lineX);
//         speedY = (int)(-lineSpeed * lineY);
//         driveXY(speedX, speedY, (int)deltaAngle);
//         return;
//     }

//     // если ворота под большим углом, пробуем забить гол вывертом дриблера
//     if (!gateInFront && gate.midAngle > 20)
//     {
//         if (cameraAngle > 120)
//         {
//             //print($">120, rotate {maxTorqueWithBall}");
//             drive(0, 30, 0);
//         }
//         else if (cameraAngle > 60)
//         {
//             //print($">70, rotate {50}");
//             drive(0, 50, 0);
//             dribble(0);
//         }
//         else
//         {
//             drive(cameraAngle, 30, move2gateSpeed);
//             dribble(0);
//         }
//     }
//     else if (!gateInFront && gate.midAngle < -20)
//     {
//         if (cameraAngle < -120)
//         {
//             //print($"<-120, rotate {-maxTorqueWithBall}");
//             drive(0, -30, 0);
//         }
//         else if (cameraAngle < -60)
//         {
//             //print($"<-70, rotate {-50}");
//             drive(0, -50, 0);
//             dribble(0);
//         }
//         else
//         {
//             drive(cameraAngle, -30, move2gateSpeed);
//             dribble(0);
//         }
//     }
// }