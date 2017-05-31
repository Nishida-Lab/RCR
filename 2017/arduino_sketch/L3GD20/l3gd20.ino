#include <Wire.h>
#include <L3GD20.h>

L3GD20 l3gd20;

void setup(){
  Serial.begin(9600);

  if(!l3gd20.begin(l3gd20.L3GD20_RANGE_250DPS)){
    Serial.println("Unable to initialize");
    while(true);
  }
}


void loop(){
  l3gd20.read();
  Serial.println(l3gd20.getAddress());
  Serial.print("x: "); Serial.print((int) l3gd20.data.x); Serial.print(", ");
  Serial.print("y: "); Serial.print((int) l3gd20.data.y); Serial.print(", "); 
  Serial.print("z: "); Serial.println((int) l3gd20.data.z);
  delay(300); 
}