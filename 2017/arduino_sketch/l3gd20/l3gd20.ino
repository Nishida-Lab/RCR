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
  l3gd20.read();
  switch(int num){
  case 0:
    gyro_0.time = millis();
    gyro_0.x = l3gd20.data.x;
    gyro_0.y = l3gd20.data.y;
    gyro_0.z = l3gd20.data.z;
  case 1:
    gyro_1.time = millis();
    gyro_1x = l3gd20.data.x;
    gyro_1.y = l3gd20.data.y;
    gyro_1.z = l3gd20.data.z;
  case 2:
    gyro_2.time = millis();
    gyro_2.x = l3gd20.data.x;
    gyro_2.y = l3gd20.data.y;
    gyro_2.z = l3gd20.data.z;
  }
}

void setup(){
  Serial.begin(9600);

  if(!l3gd20.begin(l3gd20.L3GD20_RANGE_250DPS)){
    Serial.println("Unable to initialize");
    while(true);
  }
}

int tim = 0;

void loop(){
  int num[3] = {0,1,2};
  
  getGyro(num[tim]);
  if(++tim > 2) tim = 0;
  
  Serial.print(" gyro_0.x: "); Serial.print(gyro_0.x);
  Serial.print(" gyro_0.y: "); Serial.print(gyro_0.y);
  Serial.print(" gyro_0.z: "); Serial.print(gyro_0.z);
  Serial.println();
}