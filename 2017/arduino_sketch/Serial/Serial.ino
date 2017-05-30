void setup(){
  Serial.begin(115200);
}

void loop(){
  int serial;

  if(Serial.available() > 0){
    serial = Serial.read();
    switch(serial){
    case 0 : Serial.print(1);  break;
    case 1 : Serial.print(2);  break;
    case 2 : Serial.print(3);  break;
    case 3 : Serial.print(4);  break;
    case 4 : Serial.print(5);  break;
    case 5 : Serial.print(6);  break;
    case 6 : Serial.print(7);  break;
    case 7 : Serial.print(8);  break;
    case 8 : Serial.print(9);  break;
    case 9 : Serial.print(10); break;
    case 10: Serial.print(11); break;
    case 11: Serial.print(12); break;
    case 12: Serial.print(13); break;
    case 13: Serial.print(14); break;
    }
    Serial.flush();
  }
}
