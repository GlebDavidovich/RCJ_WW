#include <Arduino.h>
#include "Wire.h"
#include "motors.h"
#include "sensors.h"
#include "locator.h"
#include "logics.h"
#include <GyverOLED.h>
#include <EncButton.h>
//#include <SoftwareSerial.h>

#define MENU_NUM 6
#define ANOTHER_NUM 4
#define SENSORS_NUM 6

GyverOLED<SSD1306_128x64, OLED_BUFFER> oled;
EncButton<EB_TICK, 32, 35, 34> enc;

static String MENU_ITEMS[MENU_NUM] = {"Yellow: Play Forward", "Yellow: Play Goalkeeper", "Blue: Play Forward","Blue: Play Goalkeeper", "Sensors Check", "Another"};
static String ANOTHER_ITEMS[ANOTHER_NUM] = {"Speed: ", "Kick", "MPU Calibration", "Line Calibration"};
static String SENSORS_ITEMS[SENSORS_NUM] = {"MPU Degrees:", "Locator Strength:", "Locator Degrees:", "Camera Angle:", "Line angle:", "Cam height:"};
static uint8_t pointer = 0;
//SoftwareSerial Camera(36);

float kp = 2, ki = 0, kd = 0, err, errold, integral, x;

void printPointer(uint8_t point){
  oled.setCursor(0, point);
  oled.print(">");
}

void oledPrintMenu(){
  oled.clear();
  oled.home();
  for (int i = 0; i < MENU_NUM; i++){
    oled.setCursor(0, i);
    oled.print(" " + MENU_ITEMS[i]);
  }
  printPointer(pointer);
  oled.update();
}

void oledPrintSensors (){
  oled.clear();
  oled.home();
  int arr[SENSORS_NUM] = {mpuGetDegree(), getBallAngle(), getStrength(), 0, getLineAngle_Avg(), 0};
  for (int i = 0; i < SENSORS_NUM; i++){
    oled.setCursor(0, i);
    oled.print(SENSORS_ITEMS[i]);
    oled.print(arr[i]);
  }
  oled.update();
}

void oledPrintAnother(int speed, uint8_t pointer1){
  oled.clear();
  oled.home();
  for (int i = 0; i < ANOTHER_NUM; i++){
    oled.setCursor(0, i);
    oled.print(" " + ANOTHER_ITEMS[i]);
  }
  oled.setCursor(50, 0);
  oled.print(speed);
  printPointer(pointer1);
  oled.update();
}

void turnByDegree(int16_t degree) {
  err = degree - mpuGetDegree();
  integral += (0.5 * err) * ki;
  x = kp * err + integral + kd * (err - errold);
  errold = err;
  constrain(x, -255, 255);
  // Serial.print(err);
  // Serial.print("  ");
  // Serial.print(x);
  // Serial.print("  ");
  // Serial.println(mpuGetDegree());  //Serial.print("  ");
  drive(x, x, -x, -x);
}

void linePID(int16_t speed) {
  float kp = 0.08, ki = 0, kd = 1;
  if (speed > 0)
    err = - readMux(8) + readMux(11);
  else
    err = -readMux(4) + readMux(7);
  integral += (0.5 * err) * ki;
  x = kp * err + integral + kd * (err - errold);
  errold = err;
  constrain(x, -255, 255);
  // Serial.print(isLineOnSensor(6));
  // Serial.print("  ");
  // Serial.print(isLineOnSensor(8));
  // Serial.print("  ");
  // Serial.print(isLineOnSensor(9));
  // Serial.print("  ");
  // Serial.print(isLineOnSensor(11));
  // Serial.print("  ");
  Serial.print(err);
  Serial.print("  ");
  Serial.println(x);
  drive(speed - x, -speed - x, speed + x, -speed + x);
}

void lineCalibration(){
  oled.clear();
  oled.home();
  oled.print("Waiting for click...");
  oled.update();
  enc.tick();
  while (!enc.click())
    enc.tick();
  getGreen();
  delay(100);
  getGreen();
  oled.clear();
  oled.home();
  oled.print("Green calibration done");
  oled.update();

  whiteTo0();
  
  while (!enc.click()){
    getWhite();
    enc.tick();
  }
  EEPROMSaveLines();
  oled.clear();
  oled.home();
  oled.print("White calibration done");
  oled.update();
}

void drive2ball(int color) {
  int st = getStrength(); 
  int angle = getBallAngle();
  int speed = 240;
  //getCamData();
  //int cam_angle = camera_angle;
  //while (!isBall()) {
    //float delta_angle = getCamData(color); /*getGyro();*/
    dribble(0);
    //float delta_angle = angle * 2;
    float delta_angle = -mpuGetDegree();
    float delta = 0;
    double k = 0.8;
    // if (abs(angle) <= 45){
    //   k = 3;
    // }
    if (angle >= 5) {
      //delta = sqrt(st) * k;
      delta = st * k;
      //speed = 150;
    }
    else if (angle <= -5) {
      //delta = -sqrt(st) * k;
       delta = -st * k;
      // speed = 150;
    }
    else {
      delta = 0;
      // speed = 230;
    }
    int16_t line_angle = getLineAngle_Avg(); 

    if (line_angle == 360) {
      drive(angle + delta, delta_angle, speed);
      //drive(0, delta_angle, 230);
    }
    else {
      if (line_angle == 0){
        drive(180, 200);
      }
      else
        drive(line_angle - line_angle/abs(line_angle) * 180, 200);
    }
    angle = getBallAngle();
    st = getStrength();
  //}
  drive(0,0,0,0);
  dribble(45);
  // delay(2000);
  // while (abs(mpuGetDegree()) >= 30 )
  //   turnByDegree(0);
  // kick();
}

void move2gate(int_fast32_t color) {
  //uint64_t  captureTime = millis();
  //  // angle from cam
  //  turn2gate();
  //  float cam_angle = getCamData()[0];
  //  while (IRLocator360::ReadStrenght() > 120 || IRLocator360::ReadStrenght_600() > 120){
  //    //float cam_angle = getCamData()[0];
  //    drive(cam_angle);
  //  }
  //while (/*getStrength() > 135*/  abs(getBallAngle()) <= 25) {
    dribble(45);
    float camera_angle = getCamData(color);
    int line_angle = getLineAngle_Avg(); 
    int cam_height = getCamHeight();
    if (line_angle == 360) {
      drive(camera_angle,camera_angle, 200);
    }
    else {
      drive(line_angle - line_angle/abs(line_angle) * 180, 200);
    }
    
    if (cam_height >= 87) {
      kick();
      // drive(180);
      // delay(250);
      // drive(0, 0, 0, 0);
      //break;
    }
  //}
  dribble(0);
  //drive(0,0,0,0);
}

void playForward(int color){
  oled.clear();
  oled.update();
  while (true){
    drive2ball(color);
    //turnByDegree(0);

    move2gate(color);
    //enc.tick();
    // oled.clear();
    // oled.home();
    // oled.print(getCamData(0));
    // oled.setCursor(0, 1);
    // oled.print(getCamHeight());
    // oled.update();
  }
  oledPrintMenu();
}

void drive2ballGoalkeeper() {
  int st = getStrength(); 
  int angle = getBallAngleGoalkeeper();
  int speed = 170;
  //getCamData();
  //int cam_angle = camera_angle;
  while (!isBallGoalkeeper()) {
    //float delta_angle = getCamData(color); /*getGyro();*/
    //dribble(0);
    //float delta_angle = angle * 2;
    float delta_angle = -mpuGetDegree();
    float delta = 0;
    double k = 2;
    // if (abs(angle) <= 45){
    //   k = 3;
    // }
    if (angle >= 5) {
      //delta = sqrt(st) * k;
      delta = st * k;
      //speed = 150;
    }
    else if (angle <= 5) {
      //delta = -sqrt(st) * k;
       delta = -st * k;
      // speed = 150;
    }
    else {
      delta = 0;
      // speed = 230;
    }
    int16_t line_angle = getLineAngle_Avg(); 

    if (line_angle == 360) {
      drive(angle + delta, delta_angle, speed);
      //drive(0, delta_angle, 230);
    }
    else {
      if (line_angle == 0){
        drive(180, 200);
      }
      else
        drive(line_angle - line_angle/abs(line_angle) * 180, 200);
    }
    angle = getBallAngleGoalkeeper();
    st = getStrength();
  }
  drive(0,0,0,0);
  //dribble(45);
  // delay(2000);
  // while (abs(mpuGetDegree()) >= 30 )
  //   turnByDegree(0);
  // kick();
}

void move2gateGoalkeeper() {
  while (abs(getBallAngleGoalkeeper()) <= 25) {

    float camera_angle = -mpuGetDegree();
    int line_angle = getLineAngle_Avg();

    if (line_angle == 360) {
      drive(camera_angle,camera_angle, 200);
    }
    else {
      drive(line_angle - line_angle/abs(line_angle) * 180, 200);
    }
  }
  //drive(0,0,0,0);
}

void playGoalkeeper(){
  // oled.clear();
  // oled.update();
  // while (!enc.click()){
  //   enc.tick();
  // }
  // oledPrintMenu();
  while (true) {
    if (getStrength() < 5){
      drive(0, 0, 0);
      continue;
    }
    
    int ballAngle = getBallAngleGoalkeeper();
    int robotAngle = mpuGetDegree();

    if (abs(ballAngle + robotAngle) < 5){
      drive(0, 0, 0);
      continue;
    }

    double kp = 2.5, kd;
    int maxSpeed = 200;
    int lineSpeed = 400;
    int moveAngle = 0;
    int deltaAngle = 0;
    int speed = 0;

    int lineAngle = getLineAngle_Avg();
    Serial.print("Line: ");
    Serial.println(lineAngle);
    
    if (ballAngle + robotAngle > 90){
      moveAngle = ballAngle + 15;
    }
    else if (ballAngle + robotAngle < -90){
      moveAngle = ballAngle - 15;
    }
    else{
      // bool leftObstacle = (lineAngle <= -45 && lineAngle >= -135 && ballAngle < 0);
      // bool rightObstacle = (lineAngle >= 45 && lineAngle <= 135 && ballAngle > 0);
      if (ballAngle + robotAngle < 0){
        moveAngle = -95 - robotAngle;
      }
      else if (ballAngle + robotAngle > 0){
        moveAngle = 95 - robotAngle;
      }
      // if (leftObstacle || rightObstacle){
      //   maxSpeed = 0;
      // }
    }
    deltaAngle = -robotAngle * 2;

    //speed = maxSpeed * max(1, abs(ballAngle + robotAngle) / 90);
    // double ballDist = constrain(getStrength(), 60, 100) / 20;
    // if (getStrength() < 60){
    //   ballDist *= 10;
    // }

    //speed = kp * min(90, abs(ballAngle + robotAngle));// * ballDist;
    //speed = min(speed, maxSpeed);
    speed = maxSpeed;
    
    // double moveY = speed * cos(DEG_TO_RAD * moveAngle);
    // double moveX = speed * sin(DEG_TO_RAD * moveAngle);

    if (lineAngle != 360){
      int lineAng360 = lineAngle;
      if (lineAng360 < 0)
        lineAng360 += 360;
      
      // если мяч в углу, а объехать его мешает линия сзади - быстро едем к углу
      // int leftDiap = 100;
      // int rightDiap = 260;
      // if (lineAng360 >= rightDiap && lineAng360 <= leftDiap){
      //   if (ballAngle + robotAngle > 90){
      //     moveX += lineSpeed;
      //   }
      //   else if (ballAngle + robotAngle < -90){
      //     moveX -= lineSpeed;
      //   }
      // }
      
      // если мы в углу и двигаться к мячу мешает линия слева/справа - едем вперёд
      // leftDiap = 170;
      // rightDiap = 10;
      // if (lineAngle > 0 && (ballAngle) > 0){
      //   moveY += lineSpeed;
      // }
      // leftDiap = 350;
      // rightDiap = 190;
      // if (lineAngle * (ballAngle) > 0){
      //   moveY += lineSpeed;
      // }

      // int delta = moveAngle - lineAngle;
      // if (delta > 180){
      //   delta -= 180;
      // }
      // if (delta < -180){
      //   delta += 360;
      // }
      // if (-60 <= delta && delta <= 60){
      //   double lineX = -sin(lineAngle);
      //   double lineY = cos(lineAngle);
      //   double projection = lineX * moveX + lineY * moveY;

      //   if (projection >= 0){
      //     moveX = lineX * speed;
      //     moveY = lineY * speed;
      //   }
      //   if (projection < 0){
      //     moveX = -lineX * speed;
      //     moveY = -lineY * speed;
      //   }
        
      //   oled.print(moveX);
      //   oled.setCursor(0, 1);
      //   oled.print(moveY);
      // }

      // moveX -= lineSpeed * sin(DEG_TO_RAD * lineAngle);
      // moveY -= lineSpeed * cos(DEG_TO_RAD * lineAngle);
      speed = lineSpeed;
      if (lineAngle == 0){
        moveAngle = 180;
      }
      else
        moveAngle = lineAngle - lineAngle/abs(lineAngle) * 180;
    }

    // if (getStrength() < 70){
    //   if (lineAngle == 360){
    //     moveY -= 30;
    //   }
    //   // else if (!isLineBehind()){
    //   //   moveY += 50;
    //   // }
    // }
    // speed = sqrt(moveX * moveX + moveY * moveY);
    // moveAngle = atan2(moveX, moveY) * RAD_TO_DEG;
    
    drive(moveAngle, deltaAngle, speed);

    // if (lineAngle > 0){
    //   speed = 50;
    // }
    // else{
    //   speed = -50;
    // }

    // linePID(speed);

    Serial.print("    move: ");
    Serial.print(moveAngle);
    Serial.print(" ");
    
    Serial.print("    speed: ");
    Serial.print(speed);
    Serial.print(" ");
      
    Serial.print("    getStrength: ");
    Serial.print(getStrength());
    Serial.print(" ");
  }
}

void playGoalkeeper2(){
  oled.clear();
  oled.update();
  while (true){
    drive2ballGoalkeeper();
    move2gateGoalkeeper();
  }
  oledPrintMenu();
}

void penaltyChallenge(int color){
  delay(6000);

  double moveAngle = 0, deltaAngle = 0, speed = 200, toBallSpeed = 100;
  double kRotate = 1, kMoveRotate = 1;
  while (abs(mpuGetDegree()) < 170)
    turnByDegree(180);
  drive(0,0,0,0);
  while (true){
    if (!isBall()){
      if (false){ //getCamHeight() < 10){
        drive(0, 10, 0);
      }
      else{
        deltaAngle = getCamData(color) * kRotate;
        int ballAngle = getBallAngleGoalkeeper();
        moveAngle = ballAngle * kMoveRotate;
        // double moveX = speed * sin(moveAngle * DEG_TO_RAD);
        // double moveY = speed * cos(moveAngle * DEG_TO_RAD);
        if (ballAngle > 10){
          // double ballMoveAngle = getBallAngle() + 15;
          moveAngle = (ballAngle + 15) * kMoveRotate;
        }
        if (ballAngle < 10){
          // double ballMoveAngle = getBallAngle() + 15;
          moveAngle = (ballAngle - 15) * kMoveRotate;
        }
        // moveX += speed * sin(ballMoveAngle * DEG_TO_RAD);
        // moveY += speed * cos(ballMoveAngle * DEG_TO_RAD);
        // speed = sqrt(moveX * moveX + moveY * moveY);
        // moveAngle = atan2(moveX, moveY);
        drive(moveAngle, deltaAngle, speed);
      }
    }
    else {
      kick();
      drive(0, 0, 0, 0);
      break;
    }
  }
}

void sensorsCheck(){
  uint32_t tt;
  while (true){
    if (millis() - tt >= 30){
      tt = millis();
      oledPrintSensors();
    }
    enc.tick();
    if (enc.hold())
      break;
  }
  oledPrintMenu();
}

void another(){
  int16_t drbl_speed = 0;
  uint64_t tmr = 0;
  uint8_t pointer1 = 0;

  oledPrintAnother(drbl_speed, pointer1);
  while (true){
    
    enc.tick();
    if (enc.right()){
      pointer1 = constrain(pointer1 + 1, 0, ANOTHER_NUM - 1);
      oledPrintAnother(drbl_speed, pointer1);
    }
    else if (enc.left()){
      pointer1 = constrain(pointer1 - 1, 0, ANOTHER_NUM - 1);
      oledPrintAnother(drbl_speed, pointer1);
    }
    else if (enc.rightH() && pointer1 == 0){
      drbl_speed++;
      constrain(drbl_speed, 0, 180);
      dribble(drbl_speed);
      oledPrintAnother(drbl_speed, pointer1);
    }
    else if (enc.leftH() && pointer1 == 0){
      drbl_speed--;
      constrain(drbl_speed, 0, 180);
      dribble(drbl_speed);
      oledPrintAnother(drbl_speed, pointer1);
    }
    else if (enc.click()){
      switch (pointer1) {
        case 0: drbl_speed = 0; dribble(0); oledPrintAnother(0, pointer1); break;
        case 1: kick(); break;
        case 2: calibration(); break;
        case 3: lineCalibration(); break;
      }
    }
    else if (enc.hold())
      break;
  }
  dribble(0);
  oledPrintMenu();
}

void setup() {
  // настройка BLE
  //set_propertis();
  // конец настройки BLE 

  Serial.begin(115200);
  //Camera.begin(115200);
  Wire.begin();

  pinMode(39, INPUT);

  motorsPins();
  mpuAndCamInit();
  locatorInit();
  setupOffsetsFromEEPROM();
  oled.init();
  oled.clear();
  oledPrintMenu();
}

void loop() {
  // while(!Camera.available());
  //   Serial.print(Camera.read());
  // for(int i = 0; i < 16; i ++){
  //   Serial.print(readMux(i));
  //   Serial.print(" ");
  //   //delay(20);
  // }

  enc.tick();
  if (enc.right()){
    pointer = constrain(pointer + 1, 0, MENU_NUM - 1);
    oledPrintMenu();
  }
  else if (enc.left()){
    pointer = constrain(pointer - 1, 0, MENU_NUM - 1);
    oledPrintMenu();
  }
  else if (enc.click()){
    switch (pointer){
      case 0: playForward(0); break;
      case 1: penaltyChallenge(0); break;
      case 2: playForward(1); break;
      case 3: playGoalkeeper2(); break;
      case 4: sensorsCheck(); break;
      case 5: another(); break;
    }
  }

  // Serial.print("data: ");
  // Wire.requestFrom(0x12, 2);
  // while(Wire.available()){
  //   Serial.print(Wire.read());
  //   Serial.print(" ");
  // }
  // Serial.println();
  // delay(1000);

  // byte error, address;
  // int nDevices = 0;

  // delay(5000);

  // Serial.println("Scanning for I2C devices ...");
  
  //Serial.println(isBall() ? "I have a BALL!!! :)" : "I don have a ball :(");

  playGoalkeeperFollowLine();
  //drive2ballSimple();

  if (Serial.available()){
    Serial.read();
    while(!Serial.available()){
      Serial.read();
    }
    Serial.read();
  }

  // uint64_t tmrtest = millis();

  // int ballAngle = getBallAngle();
  // int rotateSpeed = ballAngle * 0.3;
  // mpuGetDegree();

  // saveLineDirection();
  // int lineAngle = getLineAngle_Delayed();
  // // Serial.print("ball: ");
  // // Serial.print(ballAngle);
  // // Serial.print(' ');
  // // Serial.print(ReadStrenght_600());
  // // Serial.print(",   line: ");
  // // Serial.println(lineAngle);
  // if (lineAngle != 360){
  //   drive(goodAngle(lineAngle + 180), 40);
  // }
  // else {
  //   drive(ballAngle, rotateSpeed, 60);
  // }

  // if (isBall()){
  //   drive(0, 0, 0, 0);
  //   kick();
  //   delay(2000);
  // }
  // Serial.println(millis() - tmrtest);
}