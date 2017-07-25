#include <Wire.h>
#include <math.h>
#include "../../libraries/VL6180X/VL6180X.cpp"
#include "./setup.h"
#include "./ACCEL.h"
#include "./GYRO.h"


//#define ROTATE_OFFSET_DEBUG
//#define ACC_OFFSET_DEBUG
//#define GYRO_OFFSET_DEBUG
//#define TIME_DEBUG
//#define GYRO_DEBUG
//define GYRO_FUNC_DEBUG
//#define DEG_DEBUG
//#define ACC_DEBUG
//#define ACC_FUNC_DEBUG
//#define ACC_POS_DEBUG
//#define SRS_DEBUG
//#define ALL_DEBUG
#define RASPI_DEBUG

#define OFFSET_L 100
#define OFFSET_H 200
#define OFFSET_TIME 10000

VL6180X vl6180x_NW;
VL6180X vl6180x_N;
VL6180X vl6180x_NE;

int timing = 0;
double acc[3] = {0,0,0};
double gyro_x = 0, gyro_y = 0, gyro_z = 0;
double offset[3] = {0,0,0};
double offset_gyro[3] = {0,0,0};
double pitch = 0,roll = 0;
double now_angular_velocity_z;

double readSensor(int sensor){
  double answer = 0;
  switch(sensor){
  case 0:
  case 1:
  case 2:
  case 3:
  case 4:
  case 5:
  case 6:
    return readAnalog(sensor); //read PSD sensor
  case 7: return pos.data_x;
  case 8: return pos.data_y;
  case 9: return pos.data_z;  //call position or accel
  case 10:
    answer = vl6180x_NW.readRangeSingleMillimeters(); 
    if(vl6180x_NW.timeoutOccurred()) answer = -1; break;
  case 11:
    answer = vl6180x_N.readRangeSingleMillimeters();
    if(vl6180x_N.timeoutOccurred()) answer = -1; break;
  case 12:
    answer = vl6180x_NE.readRangeSingleMillimeters();
    if(vl6180x_NE.timeoutOccurred()) answer = -1; break; //call Short Range Sensor
  case 13:
    return deg.data_x;
  case 14:
    return deg.data_y;
  case 15:
    return deg.data_z;  //call degree
  case 16:
    return now_angular_velocity_z;
  default:
    answer = -1; break;
  }
  return answer;
}

void affine(int timing){ //affine transformation
  int i,j;
  double global_acc[3] = {0,0,0};
  double buff = 0,term_x[3] = {0,0,0},term_y[3] = {0,0,0},term_z[3] = {0,0,0};
  double deg_x, deg_y, deg_z;

  //degree to radian
  deg_x = deg.data_x * PI / 180; 
  deg_y = deg.data_y * PI / 180; 
  deg_z = deg.data_z * PI / 180;

  Serial.print(deg_x); Serial.print(" ");
  Serial.print(deg_y); Serial.print(" ");
  Serial.print(deg_z); Serial.print(" ");
  Serial.print(" : ");
  
  //rotate transformation matrix
  double affine_x[3][3] = {{1, 0, 0}, {0, cos(deg_x), sin(deg_x)}, {0, -sin(deg_x), cos(deg_x)}};
  double affine_y[3][3] = {{cos(deg_y), 0, -sin(deg_y)}, {0, 1, 0}, {sin(deg_y), 0, cos(deg_y)}};
  double affine_z[3][3] = {{cos(deg_z), sin(deg_z), 0}, {-sin(deg_z), cos(deg_z), 0}, {0, 0, 1}};

  
  /*
    double affine_x[3][3] = {{1,0,0},{0,1,0},{0,0,1}}; 
    double affine_y[3][3] = {{0,0,1},{0,1,0},{1,0,0}}; 
    double affine_z[3][3] = {{1,0,0},{0,1,0},{0,0,1}}; 
  */

  //set matrix
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

//  Serial.print(global_acc[0]); Serial.print(" ");
//  Serial.print(global_acc[1]); Serial.print(" ");
//  Serial.print(global_acc[2]); Serial.print(" ");
//  Serial.print(" : ");

    //affine transformation z -> x -> y
  for(i = 0;i < 3;i++) {
    for(j = 0;j < 3;j++) buff += global_acc[j] * affine_z[j][i];
    term_z[i] = buff;
    buff = 0;
  }
  for(i = 0;i < 3;i++) global_acc[i] = term_z[i];

//  Serial.print(global_acc[0]); Serial.print(" ");
//  Serial.print(global_acc[1]); Serial.print(" ");
//  Serial.print(global_acc[2]); Serial.print(" ");
//    Serial.print(" : ");
  for(i = 0;i < 3;i++) {
    for(j = 0;j < 3;j++) buff += global_acc[j] * affine_x[j][i];
    term_x[i] = buff;
    buff = 0;
  }
  for(i = 0;i < 3;i++) global_acc[i] = term_x[i];
  
//  Serial.print(global_acc[0]); Serial.print(" ");
//  Serial.print(global_acc[1]); Serial.print(" ");
//  Serial.print(global_acc[2]); Serial.print(" ");
//    Serial.print(" : ");
  for(i = 0;i < 3;i++) {
    for(j = 0;j < 3;j++) buff += global_acc[j] * affine_y[j][i];
    term_y[i] = buff;
    buff = 0;
  }
  for(i = 0;i < 3;i++) global_acc[i] = term_y[i];

//  Serial.print(global_acc[0]); Serial.print(" ");
//  Serial.print(global_acc[1]); Serial.print(" ");
//  Serial.println(global_acc[2]);  

  //set matrix
  switch(timing){
  case 0:
    acc_0.data_x = global_acc[0];
    acc_0.data_y = global_acc[1];
    acc_0.data_z = global_acc[2]; //remove gravity
    if(abs(acc_0.data_z) < 0.25) acc_0.data_z = 0;
    break;
  case 1:
    acc_1.data_x = global_acc[0];
    acc_1.data_y = global_acc[1];
    acc_1.data_z = global_acc[2];
    if(abs(acc_1.data_z) < 0.25) acc_1.data_z = 0;
    break;
  case 2:
    acc_2.data_x = global_acc[0];
    acc_2.data_y = global_acc[1];
    acc_2.data_z = global_acc[2];
    if(abs(acc_1.data_z) < 0.25) acc_1.data_z = 0;
    break;
  }  
}

void rotate_offset(int timing){
  switch(timing){
  case 0:
    acc_0.data_y = cos(PI*roll /180) *acc_0.data_y;
    acc_0.data_z = cos(PI*roll /180) *acc_0.data_z;
    acc_0.data_x = cos(PI*pitch/180)*acc_0.data_x;
    acc_0.data_z = cos(PI*pitch/180)*acc_0.data_z;
    break;
  case 1:
    acc_1.data_y = cos(PI*roll /180) *acc_1.data_y;
    acc_1.data_z = cos(PI*roll /180) *acc_1.data_z;
    acc_1.data_x = cos(PI*pitch/180)*acc_1.data_x;
    acc_1.data_z = cos(PI*pitch/180)*acc_1.data_z;
    break;
  case 2:
    acc_2.data_y = cos(PI*roll /180) *acc_2.data_y;
    acc_2.data_z = cos(PI*roll /180) *acc_2.data_z;
    acc_2.data_x = cos(PI*pitch/180)*acc_2.data_x;
    acc_2.data_z = cos(PI*pitch/180)*acc_2.data_z;
    break;  
  }  
}

void setup(){
  int i;
  double param = 0.95;
  int count_offset = 0;
  long sum_offset[3] = {0,0,0};
  double sum_gyro[3] = {0,0,0};
  
  
  Serial.begin(9600);
  Wire.begin();
  // Serial.println("Wire begin");
  
  //pin mode setup----------------------------------------------------- 
  pinMode( 5, OUTPUT);
  pinMode( 6, OUTPUT);
  pinMode( 7, OUTPUT); //SRS_GPIO0 setup
  pinMode( 8, OUTPUT); 
  pinMode( 9, OUTPUT);
  pinMode(10, OUTPUT); 
  pinMode(11, OUTPUT); //Multiplexer setup

  //Serial.println("PIN mode Setup");
   //gyro sensor setup------------------------------------------------
  if(!l3gd20.begin(l3gd20.L3GD20_RANGE_250DPS)){ //SLAVE_ADDRESS 0x6A (106d)
    Serial.println(-1);
    while(true);
  }
  //Serial.println("GYRO setup");
  
  //short-range sensor setup--------------------------------------------
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  //Serial.println("PIN Setup LOW");
  
  digitalWrite(5, HIGH);         // Serial.println("[debug]:D5 Setup HIGH");
  vl6180x_NW.init();             // Serial.println("[debug]:vl6180x_NW Setup Start");
  vl6180x_NW.configureDefault(); // Serial.println("[debug]:vl6180x_NW Setup default");
  vl6180x_NW.setAddress(98);     // Serial.println("[debug]:vl6180x_NW Setup Addless"); //SLAVE_ADDRESS 98 
  vl6180x_NW.setTimeout(500);    // Serial.println("[debug]:D5 Setup complete");
				  
  digitalWrite(6, HIGH);         // Serial.println("[debug]:D6 Setup HIGH");
  vl6180x_N.init();              // Serial.println("[debug]:vl6180x_N Setup Start");
  vl6180x_N.configureDefault();  // Serial.println("[debug]:vl6180x_N Setup default");
  vl6180x_N.setAddress(99);      // Serial.println("[debug]:vl6180x_N Setup Addless"); //SLAVE_ADDRESS 99
  vl6180x_N.setTimeout(500);     // Serial.println("[debug]:D6 Setup complete"); 
 				  
  digitalWrite(7, HIGH);         // Serial.println("[debug]:D7 Setup HIGH");
  vl6180x_NE.init();             // Serial.println("[debug]:vl6180x_NE Setup Start");
  vl6180x_NE.configureDefault(); // Serial.println("[debug]:vl6180x_NE Setup default");
  vl6180x_NE.setAddress(100);    // Serial.println("[debug]:vl6180x_NE Setup Addless"); //SLAVE_ADDRESS 100
  vl6180x_NE.setTimeout(500);    // Serial.println("[debug]:D7 Setup complete");

  //Serial.println("SRS Setup complete");
  //Serial.println("All Setup complete");


    while(millis() < OFFSET_TIME){//auto offset
    acc[0] = param * acc[0] + (1-param) * readAnalog(ACC_X);
    acc[1] = param * acc[1] + (1-param) * readAnalog(ACC_Y);
    acc[2] = param * acc[2] + (1-param) * readAnalog(ACC_Z);
 
    while(!l3gd20.read()) Serial.println(-1);
    gyro_x = param*gyro_x + (1-param)*l3gd20.data.x; 
    gyro_y = param*gyro_y + (1-param)*l3gd20.data.y; 
    gyro_z = param*gyro_z + (1-param)*l3gd20.data.z; 
 
    if(OFFSET_L < count_offset && count_offset <= OFFSET_H){
      for(i = 0;i < 3;i++) sum_offset[i] += acc[i];
      sum_gyro[0] += gyro_x; 
      sum_gyro[1] += gyro_y; 
      sum_gyro[2] += gyro_z; 
    }
    
    offset[0] = 4.9 * sum_offset[0]/(OFFSET_H - OFFSET_L);
    offset[1] = 4.9 * sum_offset[1]/(OFFSET_H - OFFSET_L);
    offset[2] = (4.9 * sum_offset[2] / (OFFSET_H - OFFSET_L)) - 660;

    double buffer[3];    
    buffer[0] = (offset[0] - 1662)/660;
    buffer[1] = (offset[1] - 1606)/660;
    buffer[2] = (offset[2] - 1586.65)/660;
   
    double sum_x;
    sum_x = sqrt(pow(buffer[1],2) + pow(buffer[2], 2));
    pitch = atan2(buffer[2], buffer[0]);
    roll  = atan2(buffer[2], buffer[1]);
    
    offset_gyro[0] = sum_gyro[0]/(OFFSET_H - OFFSET_L); 
    offset_gyro[1] = sum_gyro[1]/(OFFSET_H - OFFSET_L); 
    offset_gyro[2] = sum_gyro[2]/(OFFSET_H - OFFSET_L);

#ifdef ROTATE_OFFSET_DEBUG
    Serial.print  (buffer[0],10); Serial.print(" "); 
    Serial.print  (buffer[1],10); Serial.print(" ");
    Serial.print  (buffer[2],10); Serial.print(" ");
    Serial.print  (pitch); Serial.print(" "); 
    Serial.println(roll);
#endif
    
#ifdef ACC_OFFSET_DEBUG
    Serial.print  (acc[0],10); Serial.print(" "); 
    Serial.print  (acc[1],10); Serial.print(" ");
    Serial.println(acc[2],10);
#endif

#ifdef GYRO_OFFSET_DEBUG
    Serial.print  (gyro_x,10); Serial.print(" ");
    Serial.print  (gyro_y, 10); Serial.print(" ");
    Serial.println(gyro_z,10);
#endif    
    count_offset++;
  }
}

void loop(){
  int i;
  int param = 0.97;
  unsigned long time;
  //  Serial.println("loop head");
  
  while(!l3gd20.read()) Serial.println(-1); //read Gyro sensor
  acc[0] = readAnalog(ACC_X) * (1-param) + param * acc[0]; //RC filter
  acc[1] = readAnalog(ACC_Y) * (1-param) + param * acc[1];
  acc[2] = readAnalog(ACC_Z) * (1-param) + param * acc[2];
  
  time = micros();

  getGyro(timing, time, l3gd20.data.x, l3gd20.data.y, l3gd20.data.z, offset_gyro[0], offset_gyro[1], offset_gyro[2]); //get degree velocity
  if(timing == 0) now_angular_velocity_z = gyro_0.data_z;
  if(timing == 1) now_angular_velocity_z = gyro_1.data_z;
  if(timing == 2) now_angular_velocity_z = gyro_2.data_z;

  getDeg(); //get degree
 
  getAcc(timing, time, acc[0], acc[1], acc[2], offset[0], offset[1], offset[2]); //get accelaration
  rotate_offset(timing);  
  //  affine(timing); //affine transformation
  getVel(timing); //get velocity
  getPos(timing); //get position

  /*--------------Debug--------------------*/
#ifdef TIME_DEBUG 
  Serial.print(time, 10); Serial.print(" ");
  Serial.print(acc_0.time,10); Serial.print(" "); 
  Serial.print(acc_1.time,10); Serial.print(" ");
  Serial.println(acc_2.time,10);
#endif

#ifdef GYRO_DEBUG
  Serial.print(l3gd20.data.x,10); Serial.print(" "); 
  Serial.print(l3gd20.data.y,10); Serial.print(" ");
  Serial.println(l3gd20.data.z,10);  
#endif

#ifdef GYRO_FUNC_DEBUG
  Serial.print(gyro_0.data_x,10); Serial.print(" "); 
  Serial.print(gyro_0.data_y,10); Serial.print(" ");
  Serial.println(gyro_0.data_z,10);
#endif

#ifdef DEG_DEBUG
  Serial.print(deg.data_x,10); Serial.print(" ");
  Serial.print(deg.data_y,10); Serial.print(" ");
  Serial.println(deg.data_z,10);
#endif

#ifdef ACC_DEBUG
  Serial.print(acc[0]); Serial.print(" ");
  Serial.print(acc[1]); Serial.print(" ");
  Serial.println(acc[2]);
#endif

#ifdef ACC_FUNC_DEBUG
  // Serial.print(GRAVITY); Serial.print(" ");
  Serial.print(acc_0.data_x,10); Serial.print(" ");
  Serial.print(acc_0.data_y,10); Serial.print(" ");
  Serial.println(acc_0.data_z,10);  
#endif

#ifdef ACC_POS_DEBUG
  Serial.print(acc_0.data_x,10); Serial.print(" ");
  Serial.print(vel_0.data_x,10); Serial.print(" ");
  Serial.println(pos.data_x,10);  
#endif

#ifdef SRS_DEBUG
  Serial.print(readSensor(10)); Serial.print(" ");
  Serial.print(readSensor(11)); Serial.print(" ");
  Serial.println(readSensor(12));
#endif

#ifdef ALL_DEBUG
  for(i = 5;i < 7;i++){
    Serial.print(readSensor(i)); Serial.print(" ");
  }Serial.println("");
#endif
  /*----------------------------------------*/

  //  Serial.println(now_angular_velocity_z,10);

#ifdef RASPI_DEBUG
  int claim = -1;
  long serial_size = 0;
  if(Serial.available() > 0){
  claim = Serial.read();
  // Serial.println(claim - '0');
  serial_size = Serial.println(readSensor(claim), 10);
  //Serial.println(serial_size);
  Serial.flush();
  }
#endif
  
  timing++;
  if(timing > 2) timing = 0;
  //Serial.println("loop end");
}