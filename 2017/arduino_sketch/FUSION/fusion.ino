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
  Serial.println("[debug] process start");

  //short-range sensor setup
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT); //GPIO0_set
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);

  digitalWrite(5, HIGH);
  vl6180x_NW.init();
  vl6180x_NW.configureDefault();
  vl6180x_NW.setAddress(98); //SLAVE_ADDRESS 500
  vl6180x_NW.setTimeout(500);
  Serial.println("[debug] vl6180x_NW setup");
  
  digitalWrite(6, HIGH);
  vl6180x_N.init();
  vl6180x_N.configureDefault(); 
  vl6180x_N.setAddress(99);  //SLAVE_ADDRESS 600
  vl6180x_N.setTimeout(500);
  Serial.println("[debug] vl6180x_N setup");

  digitalWrite(7, HIGH);
  vl6180x_NE.init();
  vl6180x_NE.configureDefault(); 
  vl6180x_NE.setAddress(100);  //SLAVE_ADDRESS 700
  vl6180x_NE.setTimeout(500);
  Serial.println("[debug] vl6180x_NE setup");

  //gyro sensor setup
  if(!l3gd20.begin(l3gd20.L3GD20_RANGE_250DPS)){ //SLAVE_ADDRESS 0x6A
    Serial.print(-1);
    while(true);
  }
}


void loop(){
  /*
  int claim = -1;
  if(Serial.available() > 0) claim = Serial.read();
  if(claim != -1) Serial.write(readSensor(claim));
  */


  Serial.print("PSD_SW: ");  Serial.print(readSensor( 0));
  Serial.print(" PSD_W: ");  Serial.print(readSensor( 1));
  Serial.print(" PSD_NW: "); Serial.print(readSensor( 2));
  Serial.print(" PSD_N: ");  Serial.print(readSensor( 3));
  Serial.print(" PSD_NE: "); Serial.print(readSensor( 4));
  Serial.print(" PSD_E: ");  Serial.print(readSensor( 5));
  Serial.print(" PSD_SE: "); Serial.print(readSensor( 6));
  Serial.print(" ACC_X: ");  Serial.print(readSensor( 7));
  Serial.print(" ACC_Y: ");  Serial.print(readSensor( 8));
  Serial.print(" ACC_Z: ");  Serial.print(readSensor( 9));
  Serial.print(" SRS_NW: "); Serial.print(readSensor(10));
  Serial.print(" SRS_N: ");  Serial.print(readSensor(11)); 
  Serial.print(" SRS_NE: "); Serial.print(readSensor(12));
  Serial.print(" ACC_X: ");  Serial.print(readSensor(13));
  Serial.print(" ACC_Y: ");  Serial.print(readSensor(14)); 
  Serial.print(" ACC_Z: ");  Serial.print(readSensor(15)); 
  Serial.println();
}
