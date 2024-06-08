#include <Arduino.h>
#include "Wire.h"
#include "motors.h"
#include "sensors.h"
#include "locator.h"
#include <GyverOLED.h>
#include <EncButton.h>

#define MENU_NUM 6
#define ANOTHER_NUM 4
#define SENSORS_NUM 4

GyverOLED<SSD1306_128x64, OLED_BUFFER> oled;
EncButton<EB_TICK, 32, 35, 34> enc;

static String MENU_ITEMS[MENU_NUM] = {"Yellow: Play Forward", "Yellow: Play Goalkeeper", "Blue: Play Forward","Blue: Play Goalkeeper", "Sensors Check", "Another"};
static String ANOTHER_ITEMS[ANOTHER_NUM] = {"Speed: ", "Kick", "MPU Calibration", "Line Calibration"};
static String SENSORS_ITEMS[SENSORS_NUM] = {"MPU Degrees:", "Locator Strength:", "Locator Degrees:", "Camera Angle:"};
static uint8_t pointer = 0;

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
  int arr[SENSORS_NUM] = {mpuGetDegree(), getBallAngle(), getStrength(), getCamData(0)};
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
  Serial.print(err);
  Serial.print("  ");
  Serial.print(x);
  Serial.print("  ");
  Serial.println(mpuGetDegree());  //Serial.print("  ");
  drive(x, x, -x, -x);
}

void linePID(int16_t speed) {
  float kp = 0.08, ki = 0, kd = 1;
  if (speed > 0)
    err = - readMux(8) + readMux(11);
  else
    err = -readMux(4) + readMux(7);
  //err = 0;
  // if (isLineOnSensor(6) && !isLineOnSensor(8)){
  //   err += 1;
  // }
  // if (!isLineOnSensor(6) && isLineOnSensor(8)){
  //   err -= 1;
  // }
  // if (isLineOnSensor(9) && !isLineOnSensor(11)){
  //   err += 1;
  // }
  // if (!isLineOnSensor(9) && isLineOnSensor(11)){
  //   err -= 1;
  // }
  //err = ((normalizedMux(6) - normalizedMux(8)) - (normalizedMux(9) - normalizedMux(11))) * 0.5;
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

void drive2ball(char color) {
  int st = getStrength(); 
  int angle = getBallAngle();
  //getCamData();
  //int cam_angle = camera_angle;
  while (!isBall()) {
    //float delta_angle = getCamData(color); /*getGyro();*/
    dribble(0);
    //float delta_angle = angle;
    float delta_angle = -mpuGetDegree();
    float delta = 0;

    if (abs(angle) >= 10) {
      delta = (angle / abs(angle)) * st * 1.5;
    } else {
      delta = 0;
    }
    int16_t line_angle = getLineAngle_Avg(); 
    if (line_angle == 360) {
      drive(angle + delta, delta_angle, 200);
    }
    else {
      drive(line_angle - line_angle/abs(line_angle) * 180, 200);
    }
    angle = getBallAngle();
    st = getStrength();
  }
  //dribble(45);
}

void move2gate(char color) {
  //uint64_t  captureTime = millis();
  //  // angle from cam
  //  turn2gate();
  //  float cam_angle = getCamData()[0];
  //  while (IRLocator360::ReadStrenght() > 120 || IRLocator360::ReadStrenght_600() > 120){
  //    //float cam_angle = getCamData()[0];
  //    drive(cam_angle);
  //  }

  while (/*getStrength() > 135*/ abs(getBallAngle()) <= 25) {

    float camera_angle = getCamData(color);
    int line_angle = getLineAngle_Avg(); 
    int cam_height = getCamHeight();
    if (line_angle == 360) {
      drive(camera_angle,camera_angle, 200);
    }
    else {
      drive(line_angle - line_angle/abs(line_angle) * 180, 200);
    }
    
    // if (cam_height >= 75) {
    //   kick();
    //   // drive(180);
    //   // delay(250);
    //   // drive(0, 0, 0, 0);
    //   break;
    // }
  }
  //drive(0,0,0,0);
}

void playForward(char color){
  oled.clear();
  oled.update();
  while (true){
    drive2ball(color);
    //turnByDegree(0);

    move2gate(color);
    //enc.tick();
  }
  oledPrintMenu();
}

void playGoalkeeper(){
  // oled.clear();
  // oled.update();
  // while (!enc.click()){
  //   enc.tick();
  // }
  // oledPrintMenu();
  while (true) {
    if (getStrength() < 5)
      return;
    
    int ballAngle = getBallAngleGoalkeeper();
    int robotAngle = mpuGetDegree();

    if (abs(ballAngle + robotAngle) < 5){
      drive(0, 0, 0);
      return;
    }

    double kp = 1.5, kd;
    int maxSpeed = 200;
    int lineSpeed = 200;
    int moveAngle = 0;
    int deltaAngle = 0;
    int speed = 0;

    int lineAngle = getLineAngle_Avg();
    Serial.print("Line: ");
    Serial.println(lineAngle);
    
    if (ballAngle + robotAngle > 90){
      moveAngle = ballAngle + 10;
    }
    else if (ballAngle + robotAngle < -90){
      moveAngle = ballAngle - 10;
    }
    else{
      if (ballAngle + robotAngle < 0){
        moveAngle = -95;
      }
      else if (ballAngle + robotAngle > 0){
        moveAngle = 95;
      }
    }
    deltaAngle = -robotAngle;

    //speed = maxSpeed * max(1, abs(ballAngle + robotAngle) / 90);
    double ballDist = constrain(getStrength(), 60, 100) / 20;
    // if (getStrength() < 60){
    //   ballDist *= 10;
    // }

    speed = kp * min(90, abs(ballAngle + robotAngle)) * ballDist;
    speed = min(speed, maxSpeed);
    
    double moveY = speed * cos(DEG_TO_RAD * moveAngle);
    double moveX = speed * sin(DEG_TO_RAD * moveAngle);
    if (lineAngle != 360){
      int lineAng360 = lineAngle;
      if (lineAng360 < 0)
        lineAng360 += 360;
      
      // если мяч в углу, а объехать его мешает линия сзади - быстро едем к углу
      int leftDiap = 100;
      int rightDiap = 260;
      if (lineAng360 >= rightDiap && lineAng360 <= leftDiap){
        if (ballAngle + robotAngle > 90){
          moveX += lineSpeed;
        }
        else if (ballAngle + robotAngle < -90){
          moveX -= lineSpeed;
        }
      }
      
      // если мы в углу и двигаться к мячу мешает линия слева/справа - едем вперёд
      leftDiap = 170;
      rightDiap = 10;
      if (lineAngle > 0 && (ballAngle) > 0){
        moveY += lineSpeed;
      }
      leftDiap = 350;
      rightDiap = 190;
      if (lineAngle < 0 && (ballAngle) < 0){
        moveY += lineSpeed;
      }

      moveX -= lineSpeed * cos(DEG_TO_RAD * lineAngle);
      moveY -= lineSpeed * sin(DEG_TO_RAD * lineAngle);
    }
    if (getStrength() < 70){
      if (lineAngle == 360){
        moveY -= 30;
      }
      // else if (!isLineBehind()){
      //   moveY += 50;
      // }
    }
    speed = sqrt(moveX * moveX + moveY * moveY);
    moveAngle = atan2(moveX, moveY) * RAD_TO_DEG;
    
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
  Serial.begin(115200);
  Wire.begin();

  motorsPins();
  mpuAndCamInit();
  locatorInit();
  setupOffsetsFromEEPROM();
  oled.init();
  oled.clear();
  oledPrintMenu();
}

void loop() {
  // for(int i = 0; i < 16; i ++){
  //   Serial.print(readMux(i));
  //   Serial.print(" ");
  //   //delay(20);
  // }
  // Serial.println();
  // Serial.println(getLineAngle_Avg());
  // int16_t line_angle = isLine(); 
  // if (line_angle == 0) 
  //   drive(90,150); 
  // else 
  //   drive(line_angle - line_angle/abs(line_angle) * 180, 100);
  //delay(100);

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
      case 1: playGoalkeeper(); break;
      case 2: playForward(1); break;
      case 3: playGoalkeeper(); break;
      case 4: sensorsCheck(); break;
      case 5: another(); break;
    }
  }

  // int line1 = getLineAngle_Avg();
  // int line2 = getLineAngle_Max();
  // Serial.print("Line angle: avg - ");
  // Serial.print(line1);
  // Serial.print(", max - ");
  // Serial.println(line2);

  //playGoalkeeper();

  // uint64_t tmr1 = millis();
  // while (millis() - tmr1 <= 1000){
  //   drive(90, -mpuGetDegree(), 200);
  // }
  // tmr1 = millis();
  // while (millis() - tmr1 <= 1000){
  //   drive(-90, -mpuGetDegree(), 200);
  // }
  //drive(0,0,100);
  //drive2ball(1);
  // Serial.print(ReadStrenght());
  // Serial.print(' ');
  // Serial.print(ReadStrenght_600());
  // Serial.print(' ');
  // Serial.println(isBall());
  // int16_t line_angle = getLineAngle_Avg(); 
  // Serial.println(line_angle);
  // Serial.print(getCamData(1));
  // Serial.print(' ');
  // Serial.println(getCamHeight());
  //linePID(-70);
  // drive(90,0,100);


}