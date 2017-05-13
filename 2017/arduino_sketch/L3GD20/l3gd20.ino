#include <Wire.h>

const int LG3GD20_ADDR = 0x6A;

unsigned int readRegister(byte reg){
  byte result = 0;

  Wire.beginTransmission(LG3GD20_ADDR);
  Serial.println("[debug]I2C Start");

  result = Wire.write(reg);
  Serial.print("[debug]write result : "); Serial.println(result);

  result = Wire.endTransmission();
  Serial.print("[debug]I2C result:"); Serial.println(result);
  Serial.println("[debug]I2C End");

  Wire.requestFrom(LG3GD20_ADDR,1);

  return Wire.read();
}

void writeRegister(byte reg, byte data){
  Wire.beginTransmission(LG3GD20_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}


void setup(){
  Wire.begin();
  Serial.println("[debug]I2C Start");

  Serial.begin(9600);
  Serial.println("[debug]Serial Start");

  int reg = readRegister(0x0F);
  Serial.print("reg:");
  Serial.println(reg);

  writeRegister(0x20, B00001111);
}

void loop(){
  int x,y,z;
  int h,l;


  l = readRegister(0x28);
  h = readRegister(0x29);
  x = h << 8 | l;
  Serial.println("[debug]Read X");

  l = readRegister(0x2A);
  h = readRegister(0x2B);
  y = h << 8 | l;
  Serial.println("[debug]Read Y");

  l = readRegister(0x2C);
  h = readRegister(0x2D);
  z = h << 8 | l;
  Serial.println("[debug]Read Z");

  Serial.print("x:");
  Serial.print(x);
  Serial.print(", y:");
  Serial.print(y);
  Serial.print(", z:");
  Serial.println(z);
  delay(300);

}