#pragma once
#include "sensors.h"
#include "TrackingCamI2C.h"
#include "Wire.h"
#include <EEPROM.h>
#include "locator.h"
#include <MPU6050_6Axis_MotionApps20.h>
#include "I2Cdev.h"


#define EN 25
#define S0 13
#define S1 12
#define S2 26
#define S3 27 
#define SIG 14

#define COLOR_YELLOW 0
#define COLOR_BLUE 1

uint8_t fifoBuffer[45];  // буфер
int offsets[6];

const int buffersize = 70;     // количество итераций калибровки
const int acel_deadzone = 10;  // точность калибровки акселерометра (по умолчанию 8)
const int gyro_deadzone = 6;   // точность калибровки гироскопа (по умолчанию 2)
int16_t ax, ay, az, gx, gy, gz;
int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
static uint32_t tmr = millis();
int16_t robot_angle;

int green[16];
int white[16];

int lastLineDirection = 360;

static uint32_t camera_timer=0;
int8_t cam_angle =0;
int cam_height;


MPU6050 mpu;
TrackingCamI2C trackingCam;

int camera_id = 51;

void mpuAndCamInit() {
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  Serial.println("Init cam...");
  //trackingCam.init(51, 400000);
  Wire.begin();
  camera_id = 51;

  pinMode(S0, OUTPUT); 
  pinMode(S1, OUTPUT); 
  pinMode(S2, OUTPUT); 
  pinMode(S3, OUTPUT);   

  digitalWrite(S0, LOW);
  digitalWrite(S1, LOW);
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);

  pinMode(EN, OUTPUT); 
  digitalWrite(EN, LOW);
}

void setupOffsetsFromEEPROM() {
  EEPROM.begin(4096);
  // ставим оффсеты из памяти
  EEPROM.get(0, offsets);
  EEPROM.get(25, green);
  EEPROM.get(90, white);
  mpu.setXAccelOffset(offsets[0]);
  mpu.setYAccelOffset(offsets[1]);
  mpu.setZAccelOffset(offsets[2]);
  mpu.setXGyroOffset(offsets[3]);
  mpu.setYGyroOffset(offsets[4]);
  mpu.setZGyroOffset(offsets[5]);

  

  Serial.println(offsets[0]);
  Serial.println(offsets[1]);
  Serial.println(offsets[2]);
  Serial.println(offsets[3]);
  Serial.println(offsets[4]);
  Serial.println(offsets[5]);
  Serial.println("green:");
  for (int i = 0; i < 16; i++){
    Serial.print(green[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println("white:");
  for (int i = 0; i < 16; i++){
    Serial.print(white[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void meansensors() {
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
  while (i < (buffersize + 101)) {  // read raw accel/gyro measurements from device
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    if (i > 100 && i <= (buffersize + 100)) {  //First 100 measures are discarded
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if (i == (buffersize + 100)) {
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
    }
    i++;
    delay(2);
  }
}

void calibration() {
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;
  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;

  while (1) {
    int ready = 0;
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);
    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);
    meansensors();
    Serial.println("...");

    if (abs(mean_ax) <= acel_deadzone) ready++;
    else ax_offset = ax_offset - mean_ax / acel_deadzone;
    if (abs(mean_ay) <= acel_deadzone) ready++;
    else ay_offset = ay_offset - mean_ay / acel_deadzone;
    if (abs(16384 - mean_az) <= acel_deadzone) ready++;
    else az_offset = az_offset + (16384 - mean_az) / acel_deadzone;
    if (abs(mean_gx) <= gyro_deadzone) ready++;
    else gx_offset = gx_offset - mean_gx / (gyro_deadzone + 1);
    if (abs(mean_gy) <= gyro_deadzone) ready++;
    else gy_offset = gy_offset - mean_gy / (gyro_deadzone + 1);
    if (abs(mean_gz) <= gyro_deadzone) ready++;
    else gz_offset = gz_offset - mean_gz / (gyro_deadzone + 1);
    if (ready == 6) break;
  }

  offsets[0] = ax_offset;
  offsets[1] = ay_offset;
  offsets[2] = az_offset;
  offsets[3] = gx_offset;
  offsets[4] = gy_offset;
  offsets[5] = gz_offset;

  EEPROM.put(0, offsets);
  EEPROM.commit();
}

int16_t mpuGetDegree() {
  if (millis() - tmr >= 11) {  // таймер на 11 мс (на всякий случай)
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      // переменные для расчёта (ypr можно вынести в глобал)
      Quaternion q;
      VectorFloat gravity;
      float ypr[3];
      // расчёты
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      #if DebugInfo
        Debug.gyroskope_angle = degrees(ypr[0]);
      #endif
      return degrees(ypr[0]);
      tmr = millis();  // сброс таймера
    }
  }
  //return robot_angle;
}

int getStrength() {
  return ReadStrenght();
}

int lastBallSeenAngle = 0;

int getBallAngle() {
  // if (getStrength() == 0){
  //   return lastBallAngle;
  // }
  int angle;
  angle = ReadHeading_1200();
  if (ReadStrenght_600() < 15) {angle = ReadHeading_1200();}
  else {angle = ReadHeading_600(); /*Serial.println(600);*/}
  int dir = 360 - (5 * angle);
  if (dir > 180) dir = dir - 360;
  // if (abs(dir) <= 50)
  //   dir += 20;
  dir = constrain(dir, -180, 180);
  // if (abs(dir <= 30))
  //   dir -= (dir / abs(dir)) * 20;
  if (dir == -180)
    return lastBallSeenAngle;
  lastBallSeenAngle = dir;
  #if DebugInfo
    Debug.ball_angle = dir;
  #endif
  return dir;
}

int getBallAngleGoalkeeper() {
  int angle;
  angle = ReadHeading_1200();
  //if (ReadStrenght_600() < 15) angle = ReadHeading_1200();
  //else angle = ReadHeading_600();
  int dir = 360 - (5 * angle);
  if (dir > 180) dir = dir - 360;
  dir = constrain(dir, -180, 180);
  return dir;
}

int lastBallTime = 0;
int ballGapTime = 200;

void updateBallCatched(){
  if (digitalRead(39)){   // добавить пин
    lastBallTime = millis();
  }
}

bool isBall() {
  //return abs(getBallAngle()) <= 20 && getStrength() >= 50;
  updateBallCatched();
  bool ans = (millis() - lastBallTime) < ballGapTime;
  #if DebugInfo
    Debug.is_ball = ans;
  #endif
  return ans;
}

bool isBallGoalkeeper(){
  return abs(getBallAngle()) <= 20 && getStrength() >= 50;
}

int8_t getCamData(int color) { //0 - yellow, 1 - blue
  if (millis() - camera_timer >= 30) {
    camera_timer = millis();
    uint8_t n = trackingCam.readBlobs(5);
    //Serial.println(n);
    if (n == 0) return 0;
    TrackingCamBlobInfo_t maxBlob;
    if (n > 1) {
      for (int i = 1; i < n; i++) {
        if (trackingCam.blob[i].area > trackingCam.blob[i-1].area && trackingCam.blob[i].type == color) 
          maxBlob = trackingCam.blob[i];
      }
    } else {
      if (trackingCam.blob[0].type == color)
        maxBlob = trackingCam.blob[0];
    }
    cam_angle = (maxBlob.cx-160)*70/320;
    cam_height = maxBlob.top;
    // Serial.print(maxBlob.type);
    // Serial.print("  ");
    //Serial.println(cam_angle);
    
    return cam_angle;
  }
  return cam_angle;
  //return 0;
}

int getCamHeight(){
  return cam_height;
}

OmniCamBlobInfo_t blueGateBlob;
OmniCamBlobInfo_t yellowGateBlob;
OmniCamBlobInfo_t obstacleBlob;

uint8_t OpenMV_ReadData(uint8_t cam_id, uint8_t addr, uint8_t len, uint8_t* resp)
{
  // Wire.beginTransmission(0x12); // transmit to device #1
  // Wire.write(addr);
  // Wire.endTransmission();

  Wire.requestFrom(0x12, len);    // request len bytes from slave device cam_id
  uint8_t idx = 0;
  while (Wire.available())
  { // slave may send less than requested
      resp[idx] = Wire.read(); // receive a byte as character
      idx++;
  }
  return 0;
}

OmniCamData_t camDataOmni;

OmniCamData_t omnicam(){
  return camDataOmni;
}

void parseOmniCamData(char* data, int len) {
    int temp = 0;
    int sign = 1;
    int parsed[14];
    memset(parsed, 0, sizeof(parsed));
    int index = 0;
    for (int i = 0; i < len && index < 14; ++i) {
        if (data[i] == ' ' || data[i] == '\0') {
            parsed[index] = temp * sign;
            index++;

            temp = 0;
            sign = 1;
            if (data[i] == '\0')
                break;
        }
        else {
            if (data[i] == '-') {
                sign = -1;
            }
            else {
                temp = temp * 10 + (data[i] & 15);
            }
        }
    }

    if (index < 14)
      return;

    camDataOmni.gates[0].left_angle = parsed[0];
    camDataOmni.gates[0].center_angle = parsed[1];
    camDataOmni.gates[0].right_angle = parsed[2];
    camDataOmni.gates[0].clos_angle = parsed[3];
    camDataOmni.gates[0].distance = parsed[4];
    camDataOmni.gates[0].width = parsed[5];
    camDataOmni.gates[0].height = parsed[6];

    camDataOmni.gates[1].left_angle = parsed[7];
    camDataOmni.gates[1].center_angle = parsed[8];
    camDataOmni.gates[1].right_angle = parsed[9];
    camDataOmni.gates[1].clos_angle = parsed[10];
    camDataOmni.gates[1].distance = parsed[11];
    camDataOmni.gates[1].width = parsed[12];
    camDataOmni.gates[1].height = parsed[13];

    // for (int i = 0; i < 14; ++i){
    //   Serial.print(parsed[i]);
    //   Serial.print(" ");
    // }
    // Serial.println();
}

char buff[512];

bool getOpenMVDataOmni(){
  if (Serial.available() > 0)
  {
    int len = 0;
    // Serial.print(millis());
    // Serial.print(" ");
    while (Serial.available() >= 512)
      Serial.read();
    while (Serial.available() > 0){
      // Serial.print(char(Serial.read()));
      buff[len++] = (char)Serial.read();
    }
    // buff[len++] = '\0';
    // Serial.println();
    parseOmniCamData(buff, len); 
    return true;
  }
  // Serial.println(buff);
  return false;
}

int readMux(int channel){
  int controlPin[] = {S0, S1, S2, S3};

  int muxChannel[16][4]={
    {0,0,0,0}, //channel 0
    {1,0,0,0}, //channel 1
    {0,1,0,0}, //channel 2
    {1,1,0,0}, //channel 3
    {0,0,1,0}, //channel 4
    {1,0,1,0}, //channel 5
    {0,1,1,0}, //channel 6
    {1,1,1,0}, //channel 7
    {0,0,0,1}, //channel 8
    {1,0,0,1}, //channel 9
    {0,1,0,1}, //channel 10
    {1,1,0,1}, //channel 11
    {0,0,1,1}, //channel 12
    {1,0,1,1}, //channel 13
    {0,1,1,1}, //channel 14
    {1,1,1,1}  //channel 15
  };

  for(int i = 0; i < 4; i ++){
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }

  return analogRead(SIG);
}

void getGreen(){ 
  for (int i = 0; i < 16; i++){ 
    green[i] = readMux(i); 
    Serial.print(green[i]); 
    Serial.print(' '); 
  } 
  
  Serial.println(); 
} 

void whiteTo0(){
  for (int i = 0; i < 16; i++){ 
    white[i] = 0;
  } 
}

void getWhite(){
  for (int i = 0; i < 16; i++){ 
    white[i] = max(white[i], readMux(i)); 
    Serial.print(white[i]); 
    Serial.print(' '); 
  } 
  
  Serial.println();
}
 
void EEPROMSaveLines(){
  EEPROM.put(25, green);
  EEPROM.put(90, white);
  EEPROM.commit();
}

bool isLineOnSensor(int sensor){
  return (readMux(sensor) - green[sensor] >= (white[sensor] - green[sensor]) * 0.5);
}

int getLineAngle_Avg(){
  double sumX = 0;
  double sumY = 0;
  int k = 0;

  double maxDist = 0;
  double angle1 = 0, angle2 = 0;
  double single = 0;
  for (int i = 0; i < 16; i++){ 
    if (isLineOnSensor(i)){ 
      double angle = (16 - i ) * 22.5; 
      //if (angle > 180) angle -= 360; 
      //Serial.print(angle);
      //Serial.print(" ");
      k++;
      sumX += sin(angle * DEG_TO_RAD);
      sumY += cos(angle * DEG_TO_RAD);
      //Serial.print(angle);
      //Serial.print("* ");
      //Serial.print("(");
      //Serial.print(sin(angle * DEG_TO_RAD));
      //Serial.print(" ");
      //Serial.print(cos(angle * DEG_TO_RAD));
      //Serial.print(") ");
      //return angle; 
    } 
  }
  if (k == 0)
    return 360;
  double averageX = sumX / k;
  double averageY = sumY / k;
  double angle = atan2(averageX, averageY) * RAD_TO_DEG;
  //Serial.print("  average = ");
  //Serial.println(angle);
  return angle;
}

int getLineAngle_Max(){
  int k = 0;
  double maxDist = 0;
  double angle1 = 0, angle2 = 0;
  double single = 0;

  int angles[16];
  for (int i = 0; i < 16; i++){ 
    if (isLineOnSensor(i)){
      double angle = (16 - i ) * 22.5; 
      angles[k] = angle;
      k++;
    }
  }
  
  //Serial.print("Sensors: ");
  for (int i = 0; i < k; i++){ 
    if (isLineOnSensor(i)){ 
      //k++;
      double a1 = angles[i];//(16 - i) * 22.5;
      single = a1;
      // Serial.print(a1);
      // Serial.print(" ");
      for (int j = 0; j < k; j++){ 
        if (i != j && isLineOnSensor(j)){ 
          double a2 = angles[j];//(16 - j) * 22.5;
          double delta = abs(a1 - a2);
          if (delta > 180){
            delta = 360 - delta;
          }
          if (delta > maxDist){
            maxDist = delta;
            angle1 = a1;
            angle2 = a2;
          }
        } 
      } 
    } 
  }
  //Serial.println();
  if (k == 0)
    return 360;
  if (k == 1){
    if (single > 180)
      single -= 360;
    return single;
  }
  double averX = (cos(DEG_TO_RAD * angle1) + cos(DEG_TO_RAD * angle2)) / 2; 
  double averY = (sin(DEG_TO_RAD * angle1) + sin(DEG_TO_RAD * angle2)) / 2; 
  int angle = atan2(averY, averX) * RAD_TO_DEG;
  return angle;
}

bool isLineBehind(){
  for (int i = 0; i < 16; i++){ 
    if (isLineOnSensor(i)){ 
      double angle = (16 - i) * 22.5;
      if (angle >= 90 && angle <= 270){
        return true;
      }
    } 
  }
  return false;
}

int getLastLineDirection(){
  return lastLineDirection;
}

bool setLastLineDirection(int value){
  if (value != 360){
    lastLineDirection = value;
  }
}

int getErr(int sensor){
  return green[sensor] + (white[sensor] - green[sensor]) * 0.5 - readMux(sensor);
}

int normalizedMux(int sensor){
  return (10. * (readMux(sensor)) / (white[sensor]));
}

int lineTimes[16];
float priority[16];
int lineActualTime = 100;

void saveLineDirection() {
    bool isLine = false;
    for (int i = 0; i < 16; i++) {
        if (isLineOnSensor(i)) {
            lineTimes[i] = millis();
            isLine = true;
        }
    }
    if (!isLine) {
        for (int i = 0; i < 16; i++) {
            lineTimes[i] = -lineActualTime;
        }
    }
}

void getLineDirection_Delayed(float& x, float& y)
{
    float sumX = 0;
    float sumY = 0;
    int k = 0;

    //Serial.print("lines ");
    for (int i = 0; i < 16; i++) {
        if (white[i] - green[i] < 700 ) //|| i == 0 || i == 1 || i == 14 || i == 15
          continue;
        int delay = lineActualTime - (millis() - lineTimes[i]) + 1;
        if (millis() - lineTimes[i] < lineActualTime && lineTimes[i] >= 0)
        {
            float ang = (16 - i) * 22.5f;
            k += delay;
            sumX += sin(ang * DEG_TO_RAD) * delay;
            sumY += cos(ang * DEG_TO_RAD) * delay;
            //Serial.print(1);
            //Serial.print(' ');
        }
        else{
          //Serial.print(0);
          //Serial.print(' ');
        }
        if (lineTimes[i] < 0)
            priority[i] = 0;
        else
            priority[i] = delay;
    }
    if (k == 0)
    {
        for (int i = 0; i < 16; i++)
        {
            lineTimes[i] = -lineActualTime;
        }
        x = 0;
        y = 0;
    }
    else
    {
        x = sumX / k;
        y = sumY / k;
        float length = sqrt(x * x + y * y);
        x /= length;
        y /= length;
    }
    //Serial.println();
}

int getLineAngle_Delayed()
{
    float x, y;
    getLineDirection_Delayed(x, y);
    if (x == 0 && y == 0)
        return 360;
    float angle = atan2(x, y) * RAD_TO_DEG;
    #if DebugInfo
      Debug.line_angle = angle;
    #endif
    return angle;
}

int getLineTime(int index)
{
    return lineTimes[index];
}