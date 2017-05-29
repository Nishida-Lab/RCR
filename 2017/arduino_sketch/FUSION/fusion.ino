#include<Wire.h>
#include<VL6180X.h>
#include<L3GD20.h>
#include<./Multiplexer.h>


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
  case 7:
  case 8:
  case 9:
    answer = readAnalog(sensor); break;
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
    answer = (int) l3gd20.data.x;
    answer = (int) l3gd20.data.y;
    answer = (int) l3gd20.data.z; break;
  default: answer = -1;
  }

  return answer;
}

void setup(){
  Serial.begin(9600);
  Wire.begin();

  //short-range sensor setup
  vl6180x_NW.init();
  vl6180x_NW.configureDefault(); //SLAVE_ADDRESS 0x212
  vl6180x_NW.setTimeout(500);

  vl6180x_N.init();
  vl6180x_N.configureDefault(); 
  vl6180x_N.setAddress(0x213);  //SLAVE_ADDRESS 0x213
  vl6180x_N.setTimeout(500);

  vl6180x_NE.init();
  vl6180x_NE.configureDefault(); 
  vl6180x_NE.setAddress(0x214);  //SLAVE_ADDRESS 0x214
  vl6180x_NE.setTimeout(500);

  //gyro sensor setup
  if(!l3gd20.begin(l3gd20.L3GD20_RANGE_250DPS)){ //SLAVE_ADDRESS 0x6A
    Serial.write(-1);
    while(true);
  }
}


void loop(){

  int claim = -1;
  if(Serial.available() > 0) claim = Serial.read();
  if(claim != -1) Serial.write(readSensor(claim));
 
//  Serial.print("VL6180X_NW: ");  Serial.print(vl6180x_NW.readRangeSingleMillimeters()); Serial.println();
//  if (vl6180x_NW.timeoutOccurred()) Serial.print(" TIMEOUT");
//  Serial.print("VL6180X_N: ");  Serial.print(vl6180x_N.readRangeSingleMillimeters()); Serial.println();
//  if (vl6180x_N.timeoutOccurred()) Serial.print(" TIMEOUT");
//  Serial.print("VL6180X_NE: ");  Serial.print(vl6180x_NE.readRangeSingleMillimeters()); Serial.println();
//  if (vl6180x_NE.timeoutOccurred()) Serial.print(" TIMEOUT");
// 
//  l3gd20.read();
//  Serial.print("L3GD20_x: "); Serial.print((int) l3gd20.data.x); Serial.print(", ");
//  Serial.print("L3GD20_y: "); Serial.print((int) l3gd20.data.y); Serial.print(", "); 
//  Serial.print("L3GD20_z: "); Serial.println((int) l3gd20.data.z); Serial.println();
//  
//  int pin;
//  Serial.print("AnalogPin: "); Serial.print(readAnalog(pin));
// 
//  delay(500); 

}
