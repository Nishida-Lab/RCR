/* This minimal example shows how to get single-shot range
measurements from the VL6180X.
The range readings are in units of mm. */

#include <Wire.h>
#include "../../libraries/VL6180X/VL6180X.cpp"
#include "../../libraries/L3GD20/L3GD20.cpp"

VL6180X vl6180x_NW, vl6180x_N, vl6180x_NE;
L3GD20 l3gd20;

double proximal_distance(int n){
  double answer = 0.1*n+0.4477;
  return answer;
}

void setup() 
{
  pinMode(5, OUTPUT); //GPIO0_1
  pinMode(6, OUTPUT); //GPIO0_2
  pinMode(7, OUTPUT);
  
  Serial.begin(9600);
  Wire.begin();
  
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);

  digitalWrite(5, HIGH);
  vl6180x_NW.init();
  vl6180x_NW.configureDefault();
  vl6180x_NW.setAddress(99);
  vl6180x_NW.setTimeout(500);


  digitalWrite(6, HIGH);
  vl6180x_N.init();
  vl6180x_N.configureDefault();
  vl6180x_N.setAddress(100);
  vl6180x_N.setTimeout(500);

  digitalWrite(7, HIGH);
  vl6180x_NE.init();
  vl6180x_NE.configureDefault();
  vl6180x_NE.setAddress(101);
  vl6180x_NE.setTimeout(500);

  if(!l3gd20.begin()){ //SLAVE_ADDRESS 0x6A (106d)
    Serial.print("failed to connect");
    while(true);
  }

}

void loop() {
 // int distance = 0;
 // distance = proximal_distance(vl6180x.readRangeSingle());
 // Serial.println(distance);
 // delay(100);
  
  Serial.print(" Z: ");   Serial.print(l3gd20.getAddress());
  
  Serial.print(" NW: ");   Serial.print(  vl6180x_NE.getAddress());  
  Serial.print(" N : ");   Serial.print(   vl6180x_N.getAddress()); 
  Serial.print(" NE: ");   Serial.println(vl6180x_NW.getAddress());
  
  
  vl6180x_NE.readRangeSingle();
  l3gd20.read();
  //Serial.print(" X: ");   Serial.print(l3gd20.data.x);  
  //Serial.print(" Y: ");   Serial.print(l3gd20.data.y); 
  //Serial.print(" Z: ");   Serial.print(l3gd20.data.z);
  // 
  //Serial.print(" NW: ");   Serial.print(vl6180x_NE.readRangeSingle());  
  //Serial.print(" N : ");   Serial.print(vl6180x_N.readRangeSingle()); 
  //Serial.print(" NE: ");   Serial.println(vl6180x_NW.readRangeSingle());
}