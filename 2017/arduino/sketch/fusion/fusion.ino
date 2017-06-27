#include<Wire.h>
#include<../../libraries/VL6180X/VL6180X.cpp>
#include"./setup.h"
#include"./ACCEL.h"
#include"./GYRO.h"

VL6180X vl6180x_NW;
VL6180X vl6180x_N;
VL6180X vl6180x_NE;

int readSensor(int sensor){
  int answer = 0;
  switch(sensor){
  case 0:
  case 1:
  case 2:
  case 3:
  case 4:
  case 5:
  case 6:
  case 7:
  case 8:
  case 9:
    return readAnalog(sensor); //read PSD & ACC sensor
  case 10:
    answer = vl6180x_NW.readRangeSingleMillimeters();
    if(vl6180x_NW.timeoutOccurred()) answer = -1; break;
  case 11:
    answer = vl6180x_N.readRangeSingleMillimeters();
    if(vl6180x_N.timeoutOccurred()) answer = -1; break;
  case 12:
    answer = vl6180x_NE.readRangeSingleMillimeters();
    if(vl6180x_NE.timeoutOccurred()) answer = -1; break;
  case 13:
    return deg.data_x;
  case 14:
    return deg.data_y;
  case 15:
    return deg.data_z;
  }
  return answer;
}

void affine(int timing){
  int i,j;
  double global_acc[3];
  double buff,term_x[3],term_y[3],term_z[3];
  double deg_x, deg_y, deg_z;

  deg_x = deg.data_x * PI / 180; 
  deg_y = deg.data_y * PI / 180; 
  deg_z = deg.data_z * PI / 180;

  double affine_x[3][3] = {{1, 0, 0}, {0, cos(deg_x), sin(deg_x)}, {0, -sin(deg_x), cos(deg_x)}};
  double affine_y[3][3] = {{cos(deg_y), 0, -sin(deg_y)}, {0, 1, 0}, {sin(deg_y), 0, cos(deg_y)}};
  double affine_z[3][3] = {{cos(deg_z), sin(deg_z), 0}, {-sin(deg_z), cos(deg_z), 0}, {0, 0, 1}};
/*
  double affine_x[3][3] = {{1,0,0},{0,1,0},{0,0,1}}; 
  double affine_y[3][3] = {{0,0,1},{0,1,0},{1,0,0}}; 
  double affine_z[3][3] = {{1,0,0},{0,1,0},{0,0,1}}; 
*/
  switch(timing){
  case 0:
    global_acc[0] = acc_0.data_x;
    global_acc[1] = acc_0.data_y;
    global_acc[2] = acc_0.data_z;
    break;
    
  case 1:
    global_acc[0] = acc_1.data_x;
    global_acc[1] = acc_1.data_y;
    global_acc[2] = acc_1.data_z;
    break;
    
  case 2:
    global_acc[0] = acc_2.data_x;
    global_acc[1] = acc_2.data_y;
    global_acc[2] = acc_2.data_z;
    break;
  }
  
  for(i = 0;i < 3;i++) {
    for(j = 0;j < 3;j++) buff += global_acc[j] * affine_x[j][i];
    term_x[i] = buff;
    buff = 0;
  }
  for(i = 0;i < 3;i++) global_acc[i] = term_x[i];
  
  for(i = 0;i < 3;i++) {
   for(j = 0;j < 3;j++) buff += global_acc[j] * affine_y[j][i];
   term_y[i] = buff;
   buff = 0;
 }
  for(i = 0;i < 3;i++) global_acc[i] = term_y[i];
  
 for(i = 0;i < 3;i++) {
   for(j = 0;j < 3;j++) buff += global_acc[j] * affine_z[j][i];
   term_z[i] = buff;
   buff = 0;
 }
   for(i = 0;i < 3;i++) global_acc[i] = term_z[i];
 
  switch(timing){
  case 0:
    acc_0.data_x = global_acc[0];
    acc_0.data_y = global_acc[1];
    acc_0.data_z = global_acc[2];
    break;
  case 1:
    acc_1.data_x = global_acc[0];
    acc_1.data_y = global_acc[1];
    acc_1.data_z = global_acc[2];
    break;
  case 2:
    acc_2.data_x = global_acc[0];
    acc_2.data_y = global_acc[1];
    acc_2.data_z = global_acc[2];
    break;
  }  
}

void setup(){
  Serial.begin(115200);
  Wire.begin();
 
  //pin mode setup----------------------------------------------------- 
  pinMode( 5, OUTPUT);
  pinMode( 6, OUTPUT);
  pinMode( 7, OUTPUT); //SRS_GPIO0 setup
  pinMode( 8, OUTPUT); 
  pinMode( 9, OUTPUT);
  pinMode(10, OUTPUT); 
  pinMode(11, OUTPUT); //Multiplexer setup

  //short-range sensor setup--------------------------------------------
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
 
  digitalWrite(5, HIGH);
  vl6180x_NW.init();
  vl6180x_NW.configureDefault();
  vl6180x_NW.setAddress(98); //SLAVE_ADDRESS 98
  vl6180x_NW.setTimeout(500);
   
  digitalWrite(6, HIGH);
  vl6180x_N.init();
  vl6180x_N.configureDefault(); 
  vl6180x_N.setAddress(99);  //SLAVE_ADDRESS 99
  vl6180x_N.setTimeout(500);
 
  digitalWrite(7, HIGH);
  vl6180x_NE.init();
  vl6180x_NE.configureDefault(); 
  vl6180x_NE.setAddress(100);  //SLAVE_ADDRESS 100
  vl6180x_NE.setTimeout(500);
 
  //gyro sensor setup------------------------------------------------
  if(!l3gd20.begin(l3gd20.L3GD20_RANGE_500DPS)){ //SLAVE_ADDRESS 0x6A (106d)
    Serial.print(-1);
    while(true);
  }
}

int timing = 0;
double acc[3] = {0,0,0};
double gyro_x = 0, gyro_y = 0, gyro_z = 0;
double param = 0.95;


void loop(){
  
  l3gd20.read();
  gyro_x = param * gyro_x + (1-param) * l3gd20.data.x;
  gyro_y = param * gyro_y + (1-param) * l3gd20.data.y;
  gyro_z = param * gyro_z + (1-param) * l3gd20.data.z;
  getGyro(timing, gyro_x, gyro_y, gyro_z);
  getDeg();
  
  acc[0] = param * acc[0] + (1-param) * readSensor(ACC_X);
  acc[1] = param * acc[1] + (1-param) * readSensor(ACC_Y);
  acc[2] = param * acc[2] + (1-param) * readSensor(ACC_Z);
  getAcc(timing,acc[0],acc[1],acc[2]);
  affine(timing);
  //getVel(timing);
  //getPos();



  
  Serial.print(deg.data_x,10); Serial.print(" ");
  Serial.print(deg.data_y,10); Serial.print(" ");
  Serial.println(deg.data_z,10);  

//  Serial.print(acc[0]); Serial.print(" ");
//  Serial.print(acc[1]); Serial.print(" ");
//  Serial.println(acc[2]);  
  
//  Serial.print(GRAVITY); Serial.print(" ");
//  Serial.print(acc_0.data_x); Serial.print(" ");
//  Serial.print(acc_0.data_y); Serial.print(" ");
//  Serial.println(acc_0.data_z);  
  
  timing++;
  if(timing > 2) timing = 0;
  delay(10);
  /*
  int claim = -1;
  if(Serial.available() > 0){
    claim = Serial.read();
    Serial.print(readSensor(claim));
  }
  Serial.flush();
  */
}
