#include "motors.h"
//#include "esp32-hal-ledc.h"

#define M1 2
#define M1PWM 15
#define M2 4
#define M2PWM 16
#define M3 19
#define M3PWM 18
#define M4 5
#define M4PWM 17
#define KICKER 23
#define DRIBBLER 33

Servo dribbler;
Cdrv8833 m1;
Cdrv8833 m2;
Cdrv8833 m3;
Cdrv8833 m4;

//const int pwmfreq = 10;

void motorsPins() {
  // pinMode(M1, OUTPUT);
  // pinMode(M2, OUTPUT);
  // pinMode(M3, OUTPUT);
  // pinMode(M4, OUTPUT);
  // pinMode(M1PWM, OUTPUT);
  // pinMode(M2PWM, OUTPUT);
  // pinMode(M3PWM, OUTPUT);
  // pinMode(M4PWM, OUTPUT);
  pinMode(KICKER, OUTPUT);
  pinMode(DRIBBLER, OUTPUT);
  // digitalWrite(M1, LOW);
  // digitalWrite(M2, LOW);
  // digitalWrite(M3, LOW);
  // digitalWrite(M4, LOW);
  // digitalWrite(M1PWM, LOW);
  // digitalWrite(M2PWM, LOW);
  // digitalWrite(M3PWM, LOW);
  // digitalWrite(M4PWM, LOW);
  m1.init(M1, M1PWM, 6, 1);
  m2.init(M2, M2PWM, 5, 1);
  m3.init(M3, M3PWM, 2, 1);
  m4.init(M4, M4PWM, 3, 1);

  
  m1.setDecayMode(drv8833DecaySlow);
  m2.setDecayMode(drv8833DecaySlow);
  m3.setDecayMode(drv8833DecaySlow);
  m4.setDecayMode(drv8833DecaySlow);

  dribbler.attach(DRIBBLER, 1000, 2000);
  dribbler.write(180);
  delay(1000);
  dribbler.write(0);
  delay(1000);
}

void singleMotorControl(uint8_t num, int16_t speed) {
  uint8_t motor;
  uint8_t motorpwm;
  switch (num) {
    case 1: motor = M1; motorpwm = M1PWM; break;
    case 2: motor = M2; motorpwm = M2PWM; break;
    case 3: motor = M3; motorpwm = M3PWM; break;
    case 4: motor = M4; motorpwm = M4PWM; break;
  }

  if (speed > 0) {
    analogWrite(motorpwm, 280 - speed);
    digitalWrite(motor, HIGH);
    // analogWrite(motor, abs(speed));
    // digitalWrite(motorpwm, LOW);
  } else {
    analogWrite(motorpwm, abs(speed));
    digitalWrite(motor, LOW);
  }
}

void drive(int16_t m1speed, int16_t m2speed, int16_t m3speed, int16_t m4speed) {
  constrain(m1speed, -255, 255);
  constrain(m2speed, -255, 255);
  constrain(m3speed, -255, 255);
  constrain(m4speed, -255, 255);

  if (m1speed == 0 && m2speed == 0 && m3speed == 0 && m4speed == 0){
    m1.brake();
    m2.brake();
    m3.brake();
    m4.brake();
    return;
  }

  m3.move(-m3speed);
  m4.move(-m4speed);
  m1.move(-m1speed);
  m2.move(-m2speed);
}

void motor_d (int speed){
  m1.move(speed);
}

void drive(float angle, int rotation_speed, int speed) {
  float k1 = sin((45 - angle) * 0.017453);
  float k2 = sin((45 + angle) * 0.017453);
  drive(speed * k2 + rotation_speed, speed * k1 + rotation_speed, speed * k2 - rotation_speed, speed * k1 - rotation_speed);
}

void drive(float angle, int speed) {
  float k1 = sin((45 - angle) * 0.017453);
  float k2 = sin((45 + angle) * 0.017453);
  // if (k1 > k2)
  //sp2 = map(sp2, -abs(k1)*baseSpeed, abs(k1)*baseSpeed , -100, 100);
  //  float k1 = sin(((MotorsAngle/2)-angle)*0.017453)/MotorsAngleSIN;
  //  float k2 = sin(((MotorsAngle/2)+angle)*0.017453)/MotorsAngleSIN;
  drive(speed * k2, speed * k1, speed * k2, speed * k1);
}

double sec45 = 1.4142135623730950488016887242097;
void driveXY(int speedX, int speedY, int rotationSpeed)
{
    drive((speedY + speedX) * sec45 + rotationSpeed,
          (speedY - speedX) * sec45 + rotationSpeed,
          (speedY + speedX) * sec45 - rotationSpeed,
          (speedY - speedX) * sec45 - rotationSpeed);
}

void kick() {
  digitalWrite(KICKER, HIGH);
  delay(200);
  digitalWrite(KICKER, LOW);
}

void dribble(uint8_t speed){
  dribbler.write(speed);
}