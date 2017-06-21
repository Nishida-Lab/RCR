#define PSD_SW 0
#define PSD_W  1
#define PSD_NW 2
#define PSD_N  3
#define PSD_NE 4
#define PSD_E  5
#define PSD_SE 6
#define ACC_X  7
#define ACC_Y  8
#define ACC_Z  9


int readAnalog(int pin){
  switch(pin){
  case PSD_SW: 
    digitalWrite(11, LOW);
    digitalWrite(10, LOW);
    digitalWrite( 9, LOW);
    digitalWrite( 8, LOW); break; //PSD_SW
  case PSD_W : 
    digitalWrite(11, HIGH);
    digitalWrite(10, LOW);
    digitalWrite( 9, LOW);
    digitalWrite( 8, LOW); break; //PSD_W
  case PSD_NW:
    digitalWrite(11, LOW);
    digitalWrite(10, HIGH);
    digitalWrite( 9, LOW);
    digitalWrite( 8, LOW); break; //PSD_NW
  case PSD_N: 
    digitalWrite(11, HIGH);
    digitalWrite(10, HIGH);
    digitalWrite( 9, LOW);
    digitalWrite( 8, LOW); break; //PSD_N
  case PSD_NE: 
    digitalWrite(11, LOW);
    digitalWrite(10, LOW);
    digitalWrite( 9, HIGH);
    digitalWrite( 8, LOW); break; //PSD_NE
  case PSD_E:
    digitalWrite(11, HIGH);
    digitalWrite(10, LOW);
    digitalWrite( 9, HIGH);
    digitalWrite( 8, LOW); break; //PSD_E
  case PSD_SE:
    digitalWrite(11, LOW);
    digitalWrite(10, HIGH);
    digitalWrite( 9, HIGH);
    digitalWrite( 8, LOW); break; //PSD_SE
  case ACC_X:
    digitalWrite(11, HIGH);
    digitalWrite(10, HIGH);
    digitalWrite( 9, HIGH);
    digitalWrite( 8, LOW); break; //Acc_x
  case ACC_Y:
    digitalWrite(11, LOW);
    digitalWrite(10, LOW);
    digitalWrite( 9, LOW);
    digitalWrite( 8, HIGH); break; //Acc_y
  case ACC_Z: 
    digitalWrite(11, HIGH);
    digitalWrite(10, LOW);
    digitalWrite( 9, LOW);
    digitalWrite( 8, HIGH); break; //Acc_z
  }

  return analogRead(0);
}

