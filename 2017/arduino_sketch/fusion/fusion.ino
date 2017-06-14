#include<Wire.h>
#include<../../arduino_libraries/VL6180X/VL6180X.h>
#include<../../arduino_libraries/L3GD20/L3GD20.h>
#include<./ACCEL.h>

VL6180X vl6180x_NW;
VL6180X vl6180x_N;
VL6180X vl6180x_NE;
L3GD20 l3gd20;

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
    return readAnalog(sensor); //read PSD sensor

    //get Position
//  case 7:
//    answer = pos_x;
//  case 8:
//    answer = pos_y;
//  case 9:
//    answer = pos_z;
    //get position
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
  }

  return answer;
}


double PSDdistance(int n){
  double answer = 45.514*pow(n*0.0049, -0.822);

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


void loop(){
  int distance = 0;
  distance = PSDdistance(readAnalog(3));
  Serial.println(distance);
  delay(100);

  /*
  int distance = 0;
  int i;
  for(i = 0;i < 7;i++){
    distance = PSDdistance(readAnalog(i));
    Serial.print(distance); Serial.print(" ");
  }
  Serial.println();
  delay(100);
  */

  /*
  int claim = -1;
  if(Serial.available() > 0){
    claim = Serial.read();
    Serial.print(readSensor(claim));
  }
  Serial.flush();
  */
}
