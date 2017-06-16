#include <Wire.h>
#include <../../arduino_libraries/L3GD20/L3GD20.cpp>

L3GD20 l3gd20;


struct DATA{
  unsigned long time;
  double x;
  double y;
  double z;
};

DATA gyro_0,gyro_1,gyro_2,vel_0,vel_1,vel_2;

void getGyro(int num, double data_x, double data_y, double data_z){
  switch(num){
  case 0:
    gyro_0.time = micros();
    gyro_0.x = data_x;
    gyro_0.y = data_y;
    gyro_0.z = data_z;

  case 1:
    gyro_1.time = micros();
    gyro_1.x = data_x;
    gyro_1.y = data_y;
    gyro_1.z = data_z;

  case 2:
    gyro_2.time = micros();
    gyro_2.x = data_x;
    gyro_2.y = data_y;
    gyro_2.z = data_z;
  }
}

void getAngularvelocity(int num, double data_x, double data_y, double data_z){
  switch(num){
  case 0:
    vel_0.time = gyro_0.time;
    vel_0.x = data_x;
    vel_0.y = data_y;
    vel_0.z = data_z;

  case 1:
    vel_1.time = gyro_1.time;
    vel_1.x = data_x;
    vel_1.y = data_y;
    vel_1.z = data_z;

  case 2:
    vel_2.time = gyro_2.time;
    vel_2.x = data_x;
    vel_2.y = data_y;
    vel_2.z = data_z;
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

int tim = 0;
double last_gyro_x = 0, last_gyro_y = 0, last_gyro_z = 0;
double last_vel_x = 0, last_vel_y = 0, last_vel_z = 0;
double last_deg_x = 0, last_deg_y = 0 ,last_deg_z = 0;
double value = 0; 

void setup(){
  Serial.begin(115200);
  Wire.begin();


  if(!l3gd20.begin()){ //SLAVE_ADDRESS 0x6A (106d)
    Serial.print(-1);
    while(true);
  }
}


void loop(){
  int num[3] = {0,1,2};
  double param = 0.97;
  double gyro_x, gyro_y, gyro_z;
  double Avel_x, Avel_y, Avel_z;
  double deg_x,deg_y,deg_z;

  l3gd20.read();
  gyro_x = param * last_gyro_x + (1-param)*l3gd20.data.x;
  last_gyro_x = gyro_x;
  gyro_y = param * last_gyro_y + (1-param)*l3gd20.data.y;
  last_gyro_y = gyro_y;
  gyro_z = param * last_gyro_z + (1-param)*l3gd20.data.z;
  last_gyro_z = gyro_z;

  getGyro(num[tim], gyro_x, gyro_y, gyro_z);
  

  Avel_x = gyro_x * 0.00875; //range 250dps
  //  Avel_x = gyro_x * 0.01750; //range 500dps
  Avel_x = param * last_vel_x + (1-param) * Avel_x;
  last_vel_x = Avel_x;
 
  Avel_y = gyro_y * 0.00875;
  //  Avel_x = gyro_y * 0.01750; //range 500dps
  Avel_y = param * last_vel_y + (1-param) * Avel_y;
  last_vel_y = Avel_y;
 
  Avel_z = gyro_z * 0.00875;
  //  Avel_x = gyro_y * 0.01750; //range 500dps
  Avel_z = param * last_vel_z + (1-param) * Avel_z;
  last_vel_z = Avel_z;

  getAngularvelocity(num[tim], Avel_x, Avel_y, Avel_z);

  deg_x = getIntegral(gyro_0.time, vel_0.x, gyro_1.time, vel_1.x, gyro_2.time, vel_2.x)/(1000*1000);
  deg_x = param * last_deg_x + (1-param) * deg_x;
  last_deg_x = deg_x;

  deg_y = getIntegral(gyro_0.time, vel_0.y, gyro_1.time, vel_1.y, gyro_2.time, vel_2.y)/(1000*1000);
  deg_y = param * last_deg_y + (1-param) * deg_y;
  last_deg_y = deg_y;

  deg_z = getIntegral(gyro_0.time, vel_0.z, gyro_1.time, vel_1.z, gyro_2.time, vel_2.z)/(1000*1000);
  deg_z = param * last_deg_z + (1-param) * deg_z;
  last_deg_z = deg_z;


  value += deg_z; 

  if(++tim > 2) tim = 0;


//  Serial.print(gyro_z); Serial.print(" ");
//  Serial.print(Avel_z); Serial.print(" ");
//  Serial.println(deg_z);
  Serial.println(value);
}