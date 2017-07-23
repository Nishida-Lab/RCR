#include <Wire.h>
#include "../../libraries/L3GD20/L3GD20.cpp"

L3GD20 l3gd20;
DATA gyro_0,gyro_1,gyro_2,deg;

void getGyro(int num, unsigned long time, double data_x, double data_y, double data_z, double offset_x, double offset_y, double offset_z){
  double k = 0.00875;  //range 250dps
  //double k = 0.0175; //range 500dps
 
  data_x = (data_x-offset_x)*k;
  data_y = (data_y-offset_y)*k;
  data_z = (data_z-offset_z)*k;

  if(abs(data_x) < 0.001) data_x = 0;
  if(abs(data_y) < 0.001) data_y = 0;
  if(abs(data_z) < 0.001) data_z = 0;
  switch(num){
  case 0:
    gyro_0.time = time;
    gyro_0.data_x = (gyro_0.data_x + gyro_1.data_x + gyro_2.data_x)/3;
    gyro_0.data_y = (gyro_0.data_y + gyro_1.data_y + gyro_2.data_y)/3;
    gyro_0.data_z = (gyro_0.data_z + gyro_1.data_z + gyro_2.data_z)/3;

    gyro_0.data_x = data_x;
    gyro_0.data_y = data_y;
    gyro_0.data_z = data_z;
    
  case 1:
    gyro_1.time = time;
    gyro_1.data_x = (gyro_0.data_x + gyro_1.data_x + gyro_2.data_x)/3;
    gyro_1.data_y = (gyro_0.data_y + gyro_1.data_y + gyro_2.data_y)/3;
    gyro_1.data_z = (gyro_0.data_z + gyro_1.data_z + gyro_2.data_z)/3;

    gyro_1.data_x = data_x;
    gyro_1.data_y = data_y;
    gyro_1.data_z = data_z;

  case 2:
    gyro_2.time = time;
    gyro_2.data_x = (gyro_0.data_x + gyro_1.data_x + gyro_2.data_x)/3;
    gyro_2.data_y = (gyro_0.data_y + gyro_1.data_y + gyro_2.data_y)/3;
    gyro_2.data_z = (gyro_0.data_z + gyro_1.data_z + gyro_2.data_z)/3;

    gyro_2.data_x = data_x;
    gyro_2.data_y = data_y;
    gyro_2.data_z = data_z;
  }

}

int count = 0;
double deg_x = 0,deg_y = 0,deg_z = 0;
void getDeg(){
  double new_deg_x,new_deg_y,new_deg_z;
  double param = 0.95;
  deg_x = getIntegral(gyro_0.time, gyro_0.data_x, gyro_1.time, gyro_1.data_x, gyro_2.time, gyro_2.data_x)/(1000*1000);
  deg_y = getIntegral(gyro_0.time, gyro_0.data_y, gyro_1.time, gyro_1.data_y, gyro_2.time, gyro_2.data_y)/(1000*1000);
  deg_z = getIntegral(gyro_0.time, gyro_0.data_z, gyro_1.time, gyro_1.data_z, gyro_2.time, gyro_2.data_z)/(1000*1000);

  if(abs(deg_x) < 0.005) deg_x = 0;
  if(abs(deg_y) < 0.005) deg_y = 0;
  if(abs(deg_z) < 0.005) deg_z = 0;
  
  if(count > 100){
    deg.data_y += deg_x*4; //transformation
    deg.data_x += deg_y*4; 
    deg.data_z += deg_z*4; 
  }

  count++;
}
