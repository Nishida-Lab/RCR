#include<Wire.h>
#include<VL6180X.h>
#include<L3GD20.h>

VL6180X vl6180x_1;
VL6180X vl6180x_2;
VL6180X vl6180x_3;
L3GD20 l3gd20;



void setup(){
  Serial.begin(9600);
  Wire.begin();

  vl6180x_1.init();
  vl6180x_1.configureDefault(); //SLAVE_ADDRESS 0x212
  vl6180x_1.setTimeout(500);

  vl6180x_2.init();
  vl6180x_2.configureDefault(); 
  vl6180x_2.setAddress(0x213);  //SLAVE_ADDRESS 0x213
  vl6180x_2.setTimeout(500);

  vl6180x_3.init();
  vl6180x_3.configureDefault(); 
  vl6180x_3.setAddress(0x214);  //SLAVE_ADDRESS 0x213
  vl6180x_3.setTimeout(500);

  if(!l3gd20.begin(l3gd20.L3GD20_RANGE_250DPS)){ //SLAVE_ADDRESS 0x6A
    Serial.println("Unable to initialize L3GD20");
    while(true);
  }
}


void loop(){
  l3gd20.read();

  Serial.print("VL6180X_1: ");  Serial.print(vl6180x_1.readRangeSingleMillimeters()); Serial.println();
  if (vl6180x_1.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.print("VL6180X_2: ");  Serial.print(vl6180x_2.readRangeSingleMillimeters()); Serial.println();
  if (vl6180x_1.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.print("VL6180X_3: ");  Serial.print(vl6180x_3.readRangeSingleMillimeters()); Serial.println();
  if (vl6180x_1.timeoutOccurred()) { Serial.print(" TIMEOUT"); }


  Serial.print("L3GD20_x: "); Serial.print((int) l3gd20.data.x); Serial.print(", ");
  Serial.print("L3GD20_y: "); Serial.print((int) l3gd20.data.y); Serial.print(", "); 
  Serial.print("L3GD20_z: "); Serial.println((int) l3gd20.data.z); Serial.println();
  
  int gp2y0a21yk = analogRead(0);
  Serial.print("GP2Y0A21YK: "); Serial.println(gp2y0a21yk);

  delay(500); 

}