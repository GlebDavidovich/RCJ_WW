#include "locator.h"

#define LOCATOR_ADDRESS 0x0E

void locatorInit() {
  Wire.begin();
  Wire.beginTransmission(LOCATOR_ADDRESS);
  Wire.write(0x00);
  Wire.endTransmission();
}

byte ReadHeading_1200() {
  Wire.beginTransmission(LOCATOR_ADDRESS);
  Wire.write(0x04);
  Wire.endTransmission();
  Wire.requestFrom(LOCATOR_ADDRESS, 1);
  byte heading = Wire.read();
  return heading;
}

byte ReadHeading_600() {
  Wire.beginTransmission(LOCATOR_ADDRESS);
  Wire.write(0x06);
  Wire.endTransmission();
  Wire.requestFrom(LOCATOR_ADDRESS, 1);
  byte heading = Wire.read();
  return heading;
}

byte ReadStrenght() {
  Wire.beginTransmission(LOCATOR_ADDRESS);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.requestFrom(LOCATOR_ADDRESS, 1);
  byte strenght = Wire.read();
  return strenght;
}

byte ReadStrenght_600() {
  Wire.beginTransmission(LOCATOR_ADDRESS);
  Wire.write(0x07);
  Wire.endTransmission();
  Wire.requestFrom(LOCATOR_ADDRESS, 1);
  byte strenght = Wire.read();
  return strenght;
}