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

uint8_t fifoBuffer[45];  // буфер
int offsets[6];

const int buffersize = 70;     // количество итераций калибровки
const int acel_deadzone = 10;  // точность калибровки акселерометра (по умолчанию 8)
const int gyro_deadzone = 6;   // точность калибровки гироскопа (по умолчанию 2)
int16_t ax, ay, az, gx, gy, gz;
int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;


MPU6050 mpu;
TrackingCamI2C trackingCam;

void mpuAndCamInit() {
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  trackingCam.init(51, 400000);

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
  EEPROM.commit();
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
  static uint32_t tmr;
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

      return degrees(ypr[0]);
      tmr = millis();  // сброс таймера
    }
  }
  return 0;
}

int getStrength() {
  return ReadStrenght();
}

int getAngle() {
  int angle;
  if (ReadStrenght_600() < 15) angle = ReadHeading_1200();
  else angle = ReadHeading_600();
  int dir = 360 - (5 * angle);
  if (dir > 180) dir = dir - 360;
  dir = constrain(dir, -180, 180);
  return dir;
}

int getAngle_600() {
  int angle;
  angle = ReadHeading_600();
  int dir = 360 - (5 * angle);
  if (dir > 180) dir = dir - 360;
  dir = constrain(dir, -180, 180);
  return dir;
}

bool isBall() {
  return (ReadStrenght_600() >= 30) && (abs(getAngle()) <= 0);
}

int8_t getCamData(char color) { //0 - yellow, 1 - blue
  static uint32_t camera_timer;
  if (millis() - camera_timer >= 33) {
    camera_timer = millis();
    uint8_t n = trackingCam.readBlobs(5);
    if (n == 0) return 0;
    TrackingCamBlobInfo_t maxBlob;
    if (n > 1) {
      for (int i = 1; i < n; i++) {
        if (trackingCam.blob[i].area > trackingCam.blob[i-1].area) 
          maxBlob = trackingCam.blob[i];
      }
    } else {
      maxBlob = trackingCam.blob[0];
    }
    if (maxBlob.type != color) return 0;
    int8_t cam_angle = (maxBlob.cx-160)*70/320;
    Serial.print(maxBlob.type);
    Serial.print("  ");
    Serial.println(cam_angle);
    return cam_angle;
  }
  return 0;
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