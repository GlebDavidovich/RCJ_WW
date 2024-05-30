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
  int arr[SENSORS_NUM] = {mpuGetDegree(), getAngle(), getStrength(), getCamData(0)};
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

void drive2ball(char color) {
  int st = getStrength(); 
  int angle = getAngle();
  //getCamData();
  //int cam_angle = camera_angle;
  while (!isBall()) {
    //float delta_angle = getCamData(color); /*getGyro();*/
    float delta_angle = 0;
    float delta = 0;

    if (abs(angle) >= 10) {
      delta = (angle / abs(angle)) * st * 0.5;
    } else {
      delta = 0;
    }
    int16_t line_angle = isLine(); 
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
    angle = getAngle();
    st = getStrength();
    if (abs(mpuGetDegree()) >= 40) {
      turnByDegree(0);
    }
  }
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

  while (/*getStrength() > 135*/ abs(getAngle()) <= 25) {

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

void playGoalkeeper(char color){
  oled.clear();
  oled.update();
  while (!enc.click()){
    enc.tick();
  }
  oledPrintMenu();
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

// int znacinie = getCamData(1);
// int counter_odinakovich_znac;

void loop() {
  // for(int i = 0; i < 12; i ++){
  //   Serial.print(readMux(i));
  //   Serial.print(" ");
  //   //delay(20);
  // }
  // Serial.println(isLine());
  // int16_t line_angle = isLine(); 
  // if (line_angle == 0) 
  //   drive(90,150); 
  // else 
  //   drive(line_angle - line_angle/abs(line_angle) * 180, 100);
  //delay(100);
  //enc.tick();
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
  //     case 1: playGoalkeeper(0); break;
  //     case 2: playForward(1); break;
  //     case 3: playGoalkeeper(1); break;
  //     case 4: sensorsCheck(); break;
  //     case 5: another(); break;
  //   }
  // }
  // drive(90, getCamData(1), 100);

  int angle = getAngle();
  Serial.print(angle);
  Serial.print(" ");

  int maxSpeed = 100;
  int moveAngle = 0;
  int deltaAngle = 0;
  int speed = 0;

  int lineAngle = isLine();
  Serial.print(lineAngle);
  if (angle < -10){
    moveAngle = -100;
    deltaAngle = -10 - (abs(angle) - 10) * 0.1;
    speed = maxSpeed;
  }
  else if (angle > 10){
    moveAngle = 100;
    deltaAngle = +10 + (abs(angle) - 10) * 0.1;
    speed = maxSpeed;
  }

  Serial.print("  strength = ");
  Serial.print(getStrength());
  Serial.print("   ");
  double moveY = speed * cos(DEG_TO_RAD * moveAngle);
  double moveX = speed * sin(DEG_TO_RAD * moveAngle);
  if (isLineBehind() && getStrength() >= 60 && abs(angle) <= 10){
    Serial.print("  !!beat ball!! ");
    moveY += getStrength() * 2;
  }
  else{
    if (!isLineBehind()){
      if (lineAngle == 360){
        Serial.print("  !!back to line!! ");
        moveY -= 50;
      }
      else{
        Serial.print("  !!forward to line!! ");
        moveY += 50;
      }
    }
  }
  speed = sqrt(moveX * moveX + moveY * moveY);
  moveAngle = atan2(moveX, moveY) * RAD_TO_DEG;

  if (abs(angle) < 10)
    deltaAngle = 0;
  if (moveAngle < 0)
    moveAngle += 360;
  drive(moveAngle, deltaAngle, speed);

  Serial.print("    move: ");
  Serial.print(moveAngle);
  Serial.print(" ");
    
  Serial.print("    rotate: ");
  Serial.print(deltaAngle);
  Serial.print(" ");

  Serial.println();

  //turnByDegree(0);
  //drive2ball(1);
  //move2gate(1);
  // int act_znacinie = getCamData(1);
  // if (act_znacinie==znacinie)
  //   counter_odinakovich_znac++;
  // else{
  //   Serial.println( act_znacinie);
  //   Serial.println ( counter_odinakovich_znac);
  //   counter_odinakovich_znac = 0;
  //   znacinie = act_znacinie;
  // }

  //getCamData(1);
  
  //drive(0, 0, -100, 100);
  //drive(90, 100);
}