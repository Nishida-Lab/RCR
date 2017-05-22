#include<Wire.h>
#include<VL6180X.h>
#include<L3GD20.h>

VL6180X vl6180x_1;
L3GD20 l3gd20;

void setup(){
  Serial.begin(9600);
  Wire.begin();

  vl6180x_1.init();
  vl6180x_1.configureDefault();
  vl6180x_1.setTimeout(500);

  if(!l3gd20.begin(l3gd20.L3GD20_RANGE_250DPS)){
    Serial.println("Unable to initialize L3GD20");
    while(true);
  }
}


void loop(){
  l3gd20.read();

  Serial.print("VL6180X_1: ");  Serial.print(vl6180x_1.readRangeSingleMillimeters());
  if (vl6180x_1.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println();


  Serial.print("L3GD20_x: "); Serial.print((int) l3gd20.data.x); Serial.print(", ");
  Serial.print("L3GD20_y: "); Serial.print((int) l3gd20.data.y); Serial.print(", "); 
  Serial.print("L3GD20_z: "); Serial.println((int) l3gd20.data.z); Serial.println();
  delay(300); 

}