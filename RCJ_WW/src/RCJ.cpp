#include <Arduino.h>
#include "Wire.h"
#include "motors.h"
#include "sensors.h"
#include "locator.h"
#include "logics.h"
#include <GyverOLED.h>
#include <EncButton.h>
#include <AirDebug.h>




#define MENU_NUM 7
#define ANOTHER_NUM 4
#define SENSORS_NUM 6

GyverOLED<SSD1306_128x64, OLED_BUFFER> oled;
EncButton<EB_TICK, 32, 35, 34> enc;

static String MENU_ITEMS[MENU_NUM] = {"Yellow: Play Forward", "Yellow: Play Goalkeeper", "Blue: Play Forward","Blue: Play Goalkeeper", "Sensors Check", "Another", "BT"};
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
  saveLineDirection();
  getOpenMVDataOmni();
  int cam_angle = omnicam().gates[0].center_angle;
  int cam_height = omnicam().gates[0].height;
  int arr[SENSORS_NUM] = {mpuGetDegree(), getBallAngle(), getStrength(), cam_angle, getLineAngle_Delayed(), cam_height};
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
  constrain(x, -100, 100);
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
    return_kick();
  }
  dribble(0);
  oledPrintMenu();
  return_kick();
}

void setup() {
  #if DebugInfo
    BT.begin("esp32");
  #endif


  Serial.setRxBufferSize(256);
  Serial.begin(115200);
  Serial2.begin(9600);
  while (Serial.available())
  {
    Serial.read();
  }
  
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

void BT_comand(){
  #if DebugInfo
  while(true){
    Debug.ball_angle=getBallAngle();
    Debug.line_angle=getLineAngle_Avg();
    Debug.gyroskope_angle = mpuGetDegree();
    getOpenMVDataOmni();
    for (int i = 0; i<16;i++)
      Debug.line_sensor[i]=isLineOnSensor(i);
    Debug.SendInfo();
  }
  #endif
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
      case 0: playForwardDribble2(0); break;
      case 1: playGoalkeeperFollowLine(); break;
      case 2: playForwardSimple(1); break;
      case 3: playGoalkeeperCamera(0); break;
      case 4: sensorsCheck(); break;
      case 5: another(); break;
      case 6: BT_comand(); break;
    }
  }

  // drive(30, 30, 30, 30);
  // dribble(0);
  
  //playForwardDribble2(0);

  // getOpenMVDataOmni();
  // dribblerGoalGoyda(1);

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

  // playGoalkeeperFollowLine();
  //playForwardSimple(0);

  //playGoalkeeperFollowLine();

  // getOpenMVDataOmni();
  // int cam_angle = omnicam().gates[1].center_angle;
  // if (cam_angle == 360){
  //   drive(0, 50, 0);
  // }
  // else{
  //   if (abs(cam_angle) > 20){
  //       drive(0, -cam_angle * 0.3, 0);
  //   }
  //   else{
  //     drive(0, 0, 0, 0);
  //   }
  // }

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