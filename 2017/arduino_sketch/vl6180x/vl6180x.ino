/* This minimal example shows how to get single-shot range
measurements from the VL6180X.
The range readings are in units of mm. */

#include <Wire.h>
#include <VL6180X.h>

VL6180X vl6180x;
VL6180X vl6180x_2;

void setup() 
{
  pinMode(3, OUTPUT); //GPIO0_1
  pinMode(5, OUTPUT); //GPIO0_2

//  pinMode(2, OUTPUT); //GPIO1_1
//  pinMode(4, OUTPUT); //GPIO1_2

  Serial.begin(9600);
  Wire.begin();
  
  digitalWrite(3, LOW);
  digitalWrite(5, LOW);

  digitalWrite(5, HIGH);
  vl6180x.init();
  vl6180x.configureDefault();
  vl6180x.setAddress(99);
  vl6180x.setTimeout(1000);


  digitalWrite(3, HIGH);
  vl6180x_2.init();
  vl6180x_2.configureDefault();
  vl6180x_2.setAddress(100);
  vl6180x_2.setTimeout(1000);
}

void loop() 
{ 
 
  //digitalWrite(2, LOW);
  Serial.print("Address: ");  Serial.print(vl6180x.getAddress());
  Serial.print(" Range: ");   Serial.print(vl6180x.readRangeSingle()); 
  //digitalWrite(2, HIGH);

  //digitalWrite(4, LOW);
  Serial.print(" Address: ");  Serial.print(vl6180x_2.getAddress());
  Serial.print(" Range: ");    Serial.println(vl6180x_2.readRangeSingle());
  //digitalWrite(4, HIGH);
}