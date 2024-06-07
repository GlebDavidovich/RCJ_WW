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
  float kp = 0.1, ki = 0, kd = 0;
  err = getErr(8);
  integral += (0.5 * err) * ki;
  x = kp * err + integral + kd * (err - errold);
  errold = err;
  constrain(x, -255, 255);
  Serial.print(err);
  Serial.print("  ");
  Serial.println(x);
  drive(speed - x, -speed - x, speed + x, -speed + x);
}

void drive2ball(char color) {
  int st = getStrength(); 
  int angle = getBallAngle();
  //getCamData();
  //int cam_angle = camera_angle;
  while (!isBall()) {
    //float delta_angle = getCamData(color); /*getGyro();*/
    float delta_angle = angle;
    float delta = 0;

    // if (abs(angle) >= 10) {
    //   delta = (angle / abs(angle)) * st * 0.5;
    // } else {
    //   delta = 0;
    // }
    //int16_t line_angle = getLineAngle_Max(); 
    int16_t line_angle = 360;
    oled.clear();
    oled.home();
    if (line_angle == 360) {
      drive(angle + delta, delta_angle, 150);
      oled.print("OK");
    }
    else {
      drive(line_angle - line_angle/abs(line_angle) * 180, 200);
      oled.print("AAAAAAAA");
    }
    oled.update();
    angle = getBallAngle();
    st = getStrength();
  }
  kick();
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
    drive(camera_angle,camera_angle, 160);
    // if (camera_height > 85) {
    //   kick();
    //   drive(180);
    //   delay(250);
    //   drive(0, 0, 0, 0);
    //   break;
    // }
  }
  //drive(0,0,0,0);
}

void playForward(char color){
  oled.clear();
  oled.update();
  while (!enc.hold()){
    drive2ball(color);
    move2gate(color);
    enc.tick();
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
  
  int ballAngle = getBallAngle();
  //Serial.print(angle);
  //Serial.print(" ");

  int maxSpeed = 150;
  int moveAngle = 0;
  int deltaAngle = 0;
  int speed = 0;

  int lineAngle = getLineAngle_Avg();
  Serial.print("Line: ");
  Serial.println(lineAngle);

  int robotAngle = mpuGetDegree();
  // if (angle < 0){
  //   deltaAngle = -(abs(angle)) * 0.1;
  // }
  // else{
  //   deltaAngle = (abs(angle)) * 0.1;
  // }
  
  if (ballAngle + robotAngle > 90){
    moveAngle = ballAngle + 10;
  }
  else if (ballAngle + robotAngle < -90){
    moveAngle = ballAngle - 10;
  }
  else{
    if (ballAngle + robotAngle < 0){
      moveAngle = -90;
    }
    else if (ballAngle + robotAngle > 0){
      moveAngle = 90;
    }
  }
  deltaAngle = -robotAngle;

  speed = maxSpeed;
  
  double moveY = speed * cos(DEG_TO_RAD * moveAngle);
  double moveX = speed * sin(DEG_TO_RAD * moveAngle);
  if (lineAngle != 360){
    int lineAng360 = lineAngle;
    if (lineAng360 < 0)
      lineAng360 += 360;
    
    // если мяч в углу, а объехать его мешает линия сзади - быстро едем к углу
    // int leftDiap = 135; //(90 + robotAngle) % 360;
    // int rightDiap = 225; // (270 + robotAngle) % 360;
    // // Serial.print("lineAng360 = ");
    // // Serial.println(lineAng360);
    // if (lineAng360 >= rightDiap && lineAng360 <= leftDiap){
    //   // Serial.print("FRONT EXCEPTION   angle = ");
    //   // Serial.println();
    //   deltaAngle = 10;
    //   if (ballAngle + robotAngle > 90){
    //     moveX += maxSpeed;
    //   }
    //   else if (ballAngle + robotAngle < -90){
    //     moveX -= maxSpeed;
    //   }
    // }
    
    // // если мы в углу и двигаться к мячу мешает линия слева/справа - едем вперёд
    // int leftDiap = (180 + robotAngle) % 360;
    // int rightDiap = (0 + robotAngle) % 360;
    // if (lineAng360 >= rightDiap && lineAng360 <= leftDiap && (ballAngle + robotAngle) > 90){
    //   moveY += maxSpeed;
    // }
    // leftDiap = (360 + robotAngle) % 360;
    // rightDiap = (180 + robotAngle) % 360;
    // if (lineAng360 >= rightDiap && lineAng360 <= leftDiap && (ballAngle + robotAngle) < -90){
    //   moveY += maxSpeed;
    // }

    moveX -= maxSpeed * cos(DEG_TO_RAD * lineAngle);
    moveY -= maxSpeed * sin(DEG_TO_RAD * lineAngle);
  }
  // if (getStrength() < 40){
  //   if (lineAngle == 360){
  //     moveY -= 30;
  //   }
  //   else if (!isLineBehind()){
  //     moveY += 50;
  //   }
  // }
  speed = sqrt(moveX * moveX + moveY * moveY);
  moveAngle = atan2(moveX, moveY) * RAD_TO_DEG;

  // if (angle < -10){
  //   moveAngle = -100;
  //   deltaAngle = -10 - (abs(angle)) * 0.1;
  //   speed = maxSpeed;
  // }
  // else if (angle > 10){
  //   moveAngle = 100;
  //   deltaAngle = +10 + (abs(angle)) * 0.1;
  //   speed = maxSpeed;
  // }

  // // Serial.print("  strength = ");
  // // Serial.print(getStrength());
  // // Serial.print("   ");
  // double moveY = speed * cos(DEG_TO_RAD * moveAngle);
  // double moveX = speed * sin(DEG_TO_RAD * moveAngle);
  // if (false){  //isLineBehind() && getStrength() >= 60 && abs(angle) <= 10){
  //   Serial.print("  !!beat ball!! ");
  //   moveY += getStrength() * 2;
  // }
  // else{
  //   if (!isLineBehind()){
  //     if (lineAngle == 360){
  //       //Serial.print("  !!back to line!! ");
  //       moveY -= 50;
  //     }
  //     else{
  //       //Serial.print("  !!forward to line!! ");
  //       moveY += 100;
  //     }
  //   }
  // }
  // speed = sqrt(moveX * moveX + moveY * moveY);
  // moveAngle = atan2(moveX, moveY) * RAD_TO_DEG;

  // if (abs(angle) < 10)
  //   deltaAngle = 0;
  // if (moveAngle < 0)
  //   moveAngle += 360;
  
  drive(moveAngle, deltaAngle, speed);
  //drive(90, 0, 150);

  Serial.print("    move: ");
  Serial.print(moveAngle);
  Serial.print(" ");
  
  Serial.print("    speed: ");
  Serial.print(speed);
  Serial.print(" ");
    
  Serial.print("    rotate: ");
  Serial.print(deltaAngle);
  Serial.print(" ");
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

  // enc.tick();
  // if (enc.right()){
  //   pointer = constrain(pointer + 1, 0, MENU_NUM - 1);
  //   oledPrintMenu();
  // }
  // else if (enc.left()){
  //   pointer = constrain(pointer - 1, 0, MENU_NUM - 1);
  //   oledPrintMenu();
  // }
  // else if (enc.click()){
  //   switch (pointer){
  //     case 0: playForward(0); break;
  //     case 1: playGoalkeeper(); break;
  //     case 2: playForward(1); break;
  //     case 3: playGoalkeeper(); break;
  //     case 4: sensorsCheck(); break;
  //     case 5: another(); break;
  //   }
  // }

  // int line1 = getLineAngle_Avg();
  // int line2 = getLineAngle_Max();
  // Serial.print("Line angle: avg - ");
  // Serial.print(line1);
  // Serial.print(", max - ");
  // Serial.println(line2);

  // playGoalkeeper();

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
  Serial.print(ReadStrenght());
  Serial.print(' ');
  Serial.println(ReadStrenght_600());
 //linePID(50);

}