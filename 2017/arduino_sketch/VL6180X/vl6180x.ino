/* This minimal example shows how to get single-shot range
measurements from the VL6180X.
The range readings are in units of mm. */

#include <Wire.h>
#include <VL6180X.h>

VL6180X vl6180x;

void setup() 
{
  Serial.begin(9600);
  Wire.begin();
  
  vl6180x.init();
  vl6180x.configureDefault();
  vl6180x.setTimeout(500);
}

void loop() 
{ 
  Serial.print(vl6180x.readRangeSingleMillimeters());
  if (vl6180x.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  
  Serial.println();
}