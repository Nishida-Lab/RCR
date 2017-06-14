#include <Wire.h>
#include <L3GD20.h>

L3GD20 l3gd20;

struct GYRO{
  unsigned long time;
  float x;
  float y;
  float z;
};

GYRO gyro_0,gyro_1,gyro_2;

void getGyro(int num){
  switch(int num){
  case 0:
    gyro_0.time = millis();
    gyro_0.x = gyro_x;
    gyro_0.y = gyro_y;
    gyro_0.z = gyro_z;
  case 1:
    gyro_1.time = millis();
    gyro_1.x = gyro_x;
    gyro_1.y = gyro_y;
    gyro_1.z = gyro_z;
  case 2:
    gyro_2.time = millis();
    gyro_2.x = gyro_x;
    gyro_2.y = gyro_y;
    gyro_2.z = gyro_z;
  }
}

void setup(){
  Serial.begin(9600);

  if(!l3gd20.begin(l3gd20.L3GD20_RANGE_500DPS)){
    Serial.println("Unable to initialize");
    while(true);
  }
}

int tim = 0;
double last_gyro_x = 0, last_gyro_y = 0, last_gyro_z = 0;
double gyro_x, gyro_y, gyro_z;

void loop(){
  int num[3] = {0,1,2};
  double param = 0.97;

  l3gd20.read();
  gyro_x = param * last_gyro_x + (1-param)*l3gd20.data.x;
  last_gyro_x = gyro_x;
  gyro_y = param * last_gyro_y + (1-param)*l3gd20.data.y;
  last_gyro_y = gyro_y;
  gyro_z = param * last_gyro_z + (1-param)*l3gd20.data.z;
  last_gyro_z = gyro_z;

  getGyro(num[tim]);
  
  int value = 0;

  if(tim == 0) value = gyro_0.x;
  if(tim == 1) value = gyro_1.x;
  if(tim == 2) value = gyro_2.x;

  if(++tim > 2) tim = 0;

  Serial.print(value);
  Serial.println();
}