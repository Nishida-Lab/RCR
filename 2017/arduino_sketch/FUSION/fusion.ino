#include<Wire.h>
#include<VL6180X.h>
#include<L3GD20.h>
#include<./ACCEL.h>

VL6180X vl6180x_NW;
VL6180X vl6180x_N;
VL6180X vl6180x_NE;
L3GD20 l3gd20;

unsigned long time_now = 0;
unsigned long time_last = 0;
unsigned long time_past = 0;

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
    answer = readAnalog(sensor); break; //read PSD sensor
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
    l3gd20.read();
    answer = (int) l3gd20.data.x; break;
  case 14:
    l3gd20.read();
    answer = (int) l3gd20.data.y; break;
  case 15:
    l3gd20.read();
    answer = (int) l3gd20.data.z; break;
  default: answer = -1;
  }

  return answer;
}


void setup(){
  Serial.begin(9600);
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


int tim = 0;

void loop(){
  int num[3] = {0,1,2};
  int claim = -1;

  getAcc(num[tim]);
  tim++;
  if(tim > 2) tim = 0;

  if(Serial.available() > 0) claim = Serial.read();
  if(claim != -1){
    Serial.print(readSensor(claim));
    Serial.flush();
  }
}