#include "logics.h"

//[Header("Управляющие переменные")]
float timer = 0;
int stateGame = 0;

//[Header("Игровая логика")]
float moveAngle = 0;
float deltaAngle = 0;
int speed = 0;

int lineSpeed = 400;
float ballAngle = 0;
float lineAngle = 0;
float cameraAngle = 0;

int speedX = 0;
int speedY = 0;

float isBallDiap = 10;
float isBallStrength = 120;

//[Header("Нападающий")]
int drive2ballSpeed = 30;
int move2gateSpeed = 30;
float goRoundBallDiap = 20;
float goRoundBallCoefFw = 1.f;
int timeBallHeld = 0;
int gateMovePause = 0;
float goRoundObstacleCoef = 7.f;
float rotateObstacleCoef = 0.125f;

int dribblerSpeed = 90;

float fb_kp = 3;
float fb_ki = 0;
float fb_kd = 40;
float fwBallIntegral = 0;
float fwBallPrev = 0;

int maxTorqueWithBall = 20;
int maxSpeedBackWithBall = 100;

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
float rotateSlowingCoef = 6;
float considerLineTime = 10;
float lineSensorPriority[16] = {1, 0.8, 0.4, 0.1, 0.1, 0.4, 0.8, 1, 1, 0.8, 0.4, 0.1, 0.1, 0.4, 0.8, 1};
int activeSensors = 8;
int maxGoalkeeperAngle = 70;
float goRoundBallCoefGk = 0.5f;

float g_kp = 100;
float g_ki = 0;
float g_kd = 100;

float leftIntegral = 0;
float leftPrev = 0;
float rightIntegral = 0;
float rightPrev = 0;

float gb_kp = 0.6;
float gb_ki = 0;
float gb_kd = 5;
float gkBallIntegral = 0;
float gkBallPrev = 0;

float gkReactOnBallDiap = 0;

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
    deltaAngle = ballAngle;

    // скорость движения к мячу (замедление вблизи)
    float err = isBallStrength - st;
    speed = (int)(drive2ballSpeed + (err - fwBallPrev) * fb_kd);
    ///print($"{name}:   err={err}, deltaErr={err - fwBallPrev}, speed = {speed}");
    fwBallPrev = err;
    speed = constrain(speed, -drive2ballSpeed, drive2ballSpeed);

    if (lineAngle == 360) // если нет линии - едем куда собирались
    {
        drive(ballAngle, (int)deltaAngle, speed);
    }
    else // иначе отъезжаем от линии
    {
        drive(goodAngle(lineAngle + 180), lineSpeed);
    }
    ballAngle = getBallAngle();
    st = getStrength();
}

void move2gateSimple(int color)
{
    // если потеряли мяч - переход в состояние 0 (drive2ball)
    if (!isBall())
    {
        stateGame = 0;
        return;
    }

    dribble(dribblerSpeed); // дриблим мяч

    // нужные переменные
    cameraAngle = getCamData(color);
    //lineAngle = getLineAngle_Delayed();
    float lineX, lineY;
    getLineDirection_Delayed(lineX, lineY);
    int cam_height = getCamHeight();
    int robotAngle = mpuGetDegree();

    if (lineX == 0 && lineY == 0) // если нет линии
    {
        if (cameraAngle == 360) // если не видим ворота
        {
            if (abs(robotAngle) < 10) // при этом смотрим вперёд - мы в переднем углу, едем назад
            {
                drive(180, -robotAngle, (int)(move2gateSpeed * 0.5f));
            }
            else // иначе поворачиваемся вперёд в оптимальном направлении
            {
                if (robotAngle < 0)
                    drive(0, maxTorqueWithBall, 0);
                else
                    drive(0, -maxTorqueWithBall, 0);
            }
        }
        else // едем к воротам
        {
            // объезжаем потенциальное препятствие

            moveAngle = cameraAngle;
            float speedX = move2gateSpeed * sin(moveAngle * DEG_TO_RAD);
            float speedY = move2gateSpeed * cos(moveAngle * DEG_TO_RAD);
            deltaAngle = cameraAngle;

            float offset = constrain(cameraAngle, -15, 15);

            float k = goRoundObstacleCoef;
            if (cam_height >= 60) // если ворота ближе - то двигаемся быстрее
                k *= 2;
            speedX += offset * k;

            driveXY((int)speedX, (int)speedY, (int)(deltaAngle));

            //drive(cameraAngle, (int)cameraAngle, move2gateSpeed);
        }
    }
    else // видим линию - уезжаем от неё
    {
        speedX = (int)(-lineSpeed * lineX);
        speedY = (int)(-lineSpeed * lineY);
        driveXY(speedX, speedY, 0);
    }

    // забиваем гол
    if (cam_height >= 87 && cameraAngle != 360 && abs(cameraAngle) < 20)
    {
        dribble(0);
        kick();
        return;
    }
}

void playForwardSimple(int color)
{
    saveLineDirection();
    if (getStrength() < 5)
    {
        drive(0, 0, 0, 0);
        return;
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
            color = (1 ^ color);
            move2gateSimple(1 ^ color);
            break;
    }
}

/// Вратарь (следование по линии) -------------------------------------------------------------------------------------

void playGoalkeeperFollowLine()
{
    saveLineDirection();
    if (getStrength() < 5)
    {
        drive(0, 0, 0, 0);
        return;
    }
    ballAngle = getBallAngle();

    int robotAngle = mpuGetDegree();
    int rotateSpeed = 0;
    speedX = speed;
    speedY = 0;

    int v1 = 0;
    int v2 = 0;
    int v3 = 0;
    int v4 = 0;

    float leftSpeed = 0;
    float rightSpeed = 0;
    int leftk = 0, rightk = 0;
    int leftSign = -1, rightSign = -1;

    Serial.print("millis: ");
    Serial.println(millis());
    for (int i = 0; i < 8; ++i)
    {
        float priority = (millis() - getLineTime(i));
        Serial.print("left sensor ");
        Serial.print(i);
        Serial.print(", line time ");
        Serial.print(getLineTime(i));
        Serial.print(",  priority = ");
        Serial.print(priority);
        if (priority < considerLineTime * 15){
            Serial.print(", start bottom part");
            leftSign = 1;
        }
        else
        {
            leftSpeed += leftSign * lineSensorPriority[i];
            Serial.print(", += ");
            Serial.print(leftSign * lineSensorPriority[i]);
            leftk += 1;
        }
        Serial.println();
    }
    Serial.println();
    for (int i = 15; i >= 8; --i)
    {
        float priority = (millis() - getLineTime(i));
        Serial.print("left sensor ");
        Serial.print(i);
        Serial.print(", line time ");
        Serial.print(getLineTime(i));
        Serial.print(",  priority = ");
        Serial.print(priority);
        Serial.print(",  need = ");
        Serial.print(considerLineTime * 15);
        if (priority < considerLineTime * 15){
            Serial.print(", start bottom part");
            rightSign = 1;
        }
        else
        {
            rightSpeed += rightSign * lineSensorPriority[i];
            Serial.print(", += ");
            Serial.print(rightSign * lineSensorPriority[i]);
            rightk += 1;
        }
        Serial.println();
    }
    Serial.println();

    if (leftk > 0)
        leftSpeed /= leftk;
    if (rightk > 0)
        rightSpeed /= rightk;
    
    Serial.print("left: ");
    Serial.print(leftSpeed);
    Serial.print(",   right: ");
    Serial.print(rightSpeed);
    Serial.println();

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
            return;
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
            return;
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
            return;
        }
    }
    else if (leftSpeed < rightSpeed && robotAngle <= -maxGoalkeeperAngle)
    {
        if (ballAngle > gkReactOnBallDiap)
            speedX = (int)(ballSpeed * speedK);
        else
        {
            drive(90, 10);
            return;
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

void drive2ballOmni()
{
    // получаем необходимые переменные
    dribble(0);
    int st = getStrength();
    ballAngle = getBallAngle();
    lineAngle = getLineAngle_Delayed();
    //OmniCamBlobInfo_t obstacle = getClosestObject(2);

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
    // speed = (int)(drive2ballSpeed + (err - fwBallPrev) * fb_kd);
    // //print($"{name}:   err={err}, deltaErr={err - fwBallPrev}, speed = {speed}");
    // fwBallPrev = err;
    // speed = constrain(speed, -drive2ballSpeed, drive2ballSpeed);
    speed = 40;

    if (lineAngle == 360) // если нет линии - едем куда собирались
    {
        // if (st < isBallStrength * 0.6f) // если мяч достаточно далеко, едем к нему с объездом препятствий
        // {
        //     speed = drive2ballSpeed;
        //     goOverObstacleOmni(speed, ballAngle, obstacle, criticalObstacleDistance * 0.5f, false);
        // }
        // else // если мяч близко, едем к нему прямо
        // {
        //     drive(ballAngle, (int)deltaAngle, speed);
        // }
        drive(ballAngle, (int)deltaAngle, speed);
    }
    else // иначе отъезжаем от линии
    {
        drive(goodAngle(lineAngle + 180), 60);
    }
    drive(ballAngle, (int)deltaAngle, speed);
    ballAngle = getBallAngle();
    st = getStrength();
}

void move2gateOmni(int color)
{
    // если потеряли мяч - переход в состояние 0 (drive2ball)
    if (!isBall())
    {
        stateGame = 0;
        return;
    }

    dribble(dribblerSpeed); // дриблим мяч

    // нужные переменные
    deltaAngle = 0;
    // OmniCamBlobInfo_t gate = getLargestObject(color);
    // float cam_distance = 1000;
    // if (gate != nullptr)
    // {
    //     cameraAngle = gate.closest;
    //     cam_distance = gate.distance;
    // }
    float lineX, lineY;

    saveLineDirection();
    getLineDirection_Delayed(lineX, lineY);
    int robotAngle = mpuGetDegree();

    // OmniCamBlobInfo_t obstacle = getClosestObject(2);
    // /*float obstacle_distance = 1000;
    // float obstacle_angle = 360;
    // if (obstacle != null)
    // {
    //     obstacle_angle = obstacle.center;
    //     obstacle_distance = obstacle.distance;
    // }*/

    // // забиваем гол
    // if (cam_distance <= 6 && gate != null)
    // {
    //     stateGame = 3;
    // }

    if (lineX == 0 && lineY == 0) // если нет линии
    {
        drive(-robotAngle, -robotAngle * 0.3, 20);
        // if (gate == null) // если не видим ворота
        // {
        //     goOverObstacleOmni(maxSpeedBackWithBall, -robotAngle, obstacle, criticalObstacleDistance);
        //     /*if (Mathf.Abs(robotAngle) < 10) // при этом смотрим вперёд - мы в переднем углу, едем назад
        //     {
        //         drive(180, -robotAngle, maxSpeedBackWithBall);
        //     }
        //     else // иначе поворачиваемся вперёд в оптимальном направлении
        //     {
        //         if (robotAngle < 0)
        //             drive(0, maxTorqueWithBall, 0);
        //         else
        //             drive(0, -maxTorqueWithBall, 0);
        //     }*/
        // }
        // else // едем к воротам
        // {
        //     // объезжаем потенциальное препятствие

        //     goOverObstacleOmni(move2gateSpeed, cameraAngle, obstacle, criticalObstacleDistance);
        // }
    }
    else // видим линию - уезжаем от неё
    {
        speedX = (int)(-lineSpeed * lineX);
        speedY = (int)(-lineSpeed * lineY);
        driveXY(speedX, speedY, (int)deltaAngle);
    }
}

// функция, чтобы ехать в опр. направлении, объезжая препятствия
void goOverObstacleOmni(float generalSpeed, float generalAngle, OmniCamBlobInfo_t obstacle, float critDist, bool rotate = true)
{
    // определяем параметры препятствия
    float obstacle_distance = 1000;
    float obstacle_angle = 360;
    if (obstacle.width != 0)
    {
        obstacle_angle = obstacle.closest;
        obstacle_distance = obstacle.distance;
    }

    // стартовые параметры для drive
    moveAngle = generalAngle;
    deltaAngle = generalAngle;
    if (abs(deltaAngle) > maxTorqueWithBall)
    {
        deltaAngle = (deltaAngle > 0) ? maxTorqueWithBall : -maxTorqueWithBall;
    }

    // если рядом препятствие
    if (obstacle.width != 0 && obstacle_distance < critDist)
    {
        float co_angle = goodAngle(obstacle_angle - generalAngle); // разность углов ворот и препятствия
        if (rotate) // если нужно отворачиваться от препятствия, то отворачиваемся
        {
            if (co_angle > 0 && co_angle < criticalObstacleAngle)
            {
                deltaAngle = -maxTorqueWithBall;
            }
            if (co_angle < 0 && co_angle > -criticalObstacleAngle)
            {
                deltaAngle = maxTorqueWithBall;
            }
        }

        if (abs(co_angle) < criticalObstacleAngle) // если препятствие не сзади
        {
            // проецируем скорость поперёк препятствия

            if (co_angle > 10)
            {
                moveAngle = goodAngle(obstacle_angle - 90);
            }
            else if (co_angle < -10)
            {
                moveAngle = goodAngle(obstacle_angle + 90);
            }
            else
            {
                moveAngle = goodAngle(obstacle_angle + 120);
            }
        }
    }

    drive((int)moveAngle, (int)deltaAngle, (int)generalSpeed);
}

void playForwardOmni(int color)
{
    saveLineDirection();
    if (getStrength() < 5)
    {
        drive(0, 0, 0, 0);
        return;
    }
    switch (stateGame)
    {
        case 0: // 0 - едем к мячу
            drive2ballOmni();
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
            move2gateOmni(1 ^ color);
            break;
        case 3: // 3 - гол дриблером
            //dribblerGoalOmni(1 ^ color);
            break;
    }
}

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