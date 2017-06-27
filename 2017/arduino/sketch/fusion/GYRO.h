#include <Wire.h>
#include "../../libraries/L3GD20/L3GD20.cpp"
//#include "struct.h"

L3GD20 l3gd20;
DATA gyro_0,gyro_1,gyro_2,deg;

void getGyro(int num, double data_x, double data_y, double data_z){
  double k = 0.00875;  //range 250dps
  //double k = 0.0175; //range 500dps
  switch(num){
  case 0:
    gyro_0.time = micros();
    gyro_0.data_x = data_x*k;
    gyro_0.data_y = data_y*k;
    gyro_0.data_z = data_z*k;
    
  case 1:
    gyro_1.time = micros();
    gyro_1.data_x = data_x*k;
    gyro_1.data_y = data_y*k;
    gyro_1.data_z = data_z*k;

  case 2:
    gyro_2.time = micros();
    gyro_2.data_x = data_x*k;
    gyro_2.data_y = data_y*k;
    gyro_2.data_z = data_z*k;
  }
}

int count = 0;
double deg_x = 0,deg_y = 0,deg_z = 0;
void getDeg(){
  double new_deg_x,new_deg_y,new_deg_z;
  double param = 0.95;
  new_deg_x = getIntegral(gyro_0.time, gyro_0.data_x, gyro_1.time, gyro_1.data_x, gyro_2.time, gyro_2.data_x)/(1000*1000);
  new_deg_y = getIntegral(gyro_0.time, gyro_0.data_y, gyro_1.time, gyro_1.data_y, gyro_2.time, gyro_2.data_y)/(1000*1000);
  new_deg_z = getIntegral(gyro_0.time, gyro_0.data_z, gyro_1.time, gyro_1.data_z, gyro_2.time, gyro_2.data_z)/(1000*1000);

  deg_x = param * deg_x + (1-param)*new_deg_x;
  deg_x = new_deg_x - deg_x;
  deg_y = param * deg_y + (1-param)*new_deg_y;
  deg_y = new_deg_y - deg_y;
  deg_z = param * deg_z + (1-param)*new_deg_z;
  deg_z = new_deg_z - deg_z;
  
  
  if(abs(deg_x) < 0.1) deg_x = 0; 
  if(abs(deg_y) < 0.1) deg_y = 0; 
  if(abs(deg_z) < 0.1) deg_z = 0; 

  if(count > 100){
    deg.data_y += deg_x*4; //transformation
    deg.data_x += deg_y*4; 
    deg.data_z += deg_z*4; 
  }

  count++;
}
