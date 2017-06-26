void setup(){
  Serial.begin(115200);
  Serial.println("serial start");
}

void loop(){
  char serial;

  if(Serial.available() > 0){
    serial = Serial.read();
    
    switch(serial){
    case  0: Serial.print("PSD_SW\n"); break;
    case  1: Serial.print("PSD_W \n"); break;
    case  2: Serial.print("PSD_NW\n"); break;
    case  3: Serial.print("PSD_N \n"); break;
    case  4: Serial.print("PSD_NE\n"); break;
    case  5: Serial.print("PSD_E \n"); break;
    case  6: Serial.print("PSD_SE\n"); break;
    case  7: Serial.print("SRS_NW\n"); break;
    case  8: Serial.print("SRS_N \n"); break;
    case  9: Serial.print("SRS_NE\n"); break;
    case 10: Serial.print("GYRO_X\n"); break;
    case 11: Serial.print("GYRO_Y\n"); break;
    case 12: Serial.print("ACC_X \n"); break;
    case 13: Serial.print("ACC_Y \n"); break;
    case 14: Serial.print("ACC_Z \n"); break;
    case 15:
      Serial.print("ACC_X"); Serial.print(" ");
      Serial.print("ACC_Y"); Serial.print(" ");
      Serial.print("ACC_Z"); Serial.print("\n"); break;
    default: Serial.print("Fxxk!!\n");
    }
    Serial.flush();
  }
}
