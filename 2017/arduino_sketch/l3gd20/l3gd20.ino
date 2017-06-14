#include <Wire.h>
#include <L3GD20.h>

int tim = 0;
double last_gyro_x = 0, last_gyro_y = 0, last_gyro_z = 0;
double gyro_x, gyro_y, gyro_z;
double deg_x,deg_y,deg_z;


L3GD20 l3gd20;

struct GYRO{
  unsigned long time;
  double x;
  double y;
  double z;
};

GYRO gyro_0,gyro_1,gyro_2;

void getGyro(int num){
  switch(num){
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


double getIntegral(unsigned long time_0, double data_0, unsigned long time_1, double data_1, unsigned long time_2, double data_2){
  double answer = 0;
  if(time_0 < time_1 && time_1 < time_2){
    answer = (data_0 + 4*data_1 + data_2)*(time_2 - time_0)/6;
  }
  else if(time_1 < time_2 && time_2 < time_0){
    answer = (data_1 + 4*data_2 + data_0)*(time_0 - time_1)/6;
  }
  else if(time_2 < time_0 && time_0 < time_1){
    answer = (data_2 + 4*data_0 + data_1)*(time_1 - time_2)/6;
  }

  return answer;
}


void setup(){
  Serial.begin(9600);

  if(!l3gd20.begin(l3gd20.L3GD20_RANGE_500DPS)){
    Serial.println("Unable to initialize");
    while(true);
  }
}


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

  deg_x = getIntegral(gyro_0.time, gyro_0.x, gyro_1.time, gyro_1.x, gyro_2.time, gyro_2.x);
  deg_y = getIntegral(gyro_0.time, gyro_0.y, gyro_1.time, gyro_1.y, gyro_2.time, gyro_2.y);
  deg_z += getIntegral(gyro_0.time, gyro_0.z, gyro_1.time, gyro_1.z, gyro_2.time, gyro_2.z);


  if(++tim > 2) tim = 0;

  Serial.print(value); Serial.print(" ");
  Serial.println(deg_z);

}