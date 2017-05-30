#include<./Multiplexer.h>

struct ACCEL{
  unsigned long time;
  int data_x;
  int data_y;
  int data_z;
};

ACCEL acc_0,acc_1,acc_2;

void getAcc(int num,char direction){
  switch(num){
  case 0:
    acc_0.time = millis();
    switch(direction){
    case 'x':
      acc_0.data_x = readAnalog(7); break;
    case 'y':
      acc_0.data_y = readAnalog(8); break;
    case 'z':
      acc_0.data_z = readAnalog(9); break;
    } break;
 
  case 1:
    acc_1.time = millis();
    switch(direction){
    case 'x':
      acc_1.data_x = readAnalog(7); break;
    case 'y':
      acc_1.data_y = readAnalog(8); break;
    case 'z':
      acc_1.data_z = readAnalog(9); break;
    }break;

  case 2:
    acc_2.time = millis();
    switch(direction){
    case 'x':
      acc_2.data_x = readAnalog(7); break;
    case 'y':
      acc_2.data_y = readAnalog(8); break;
    case 'z':
      acc_2.data_z = readAnalog(9); break;
    }break;
  }
}

int getPosition(unsigned long time_0, int acc_0, unsigned long time_1, int acc_1, unsigned long time_2, int acc_2){
  int position = 0;
  if(time_0 < time_1 && time_1 < time_2){
    position = (acc_0 + 4*acc_1 + acc_2)*(time_2 - time_0)/6;
  }
  else if(time_1 < time_2 && time_2 < time_0){
    position = (acc_1 + 4*acc_2 + acc_0)*(time_0 - time_1)/6;
  }
  else if(time_2 < time_0 && time_0 < time_1){
    position = (acc_2 + 4*acc_0 + acc_1)*(time_1 - time_2)/6;
  }

  return position;
}

