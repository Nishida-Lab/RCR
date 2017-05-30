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

int getPosition_x(ACCEL acc_0, ACCEL acc_1, ACCEL acc_2){
  int position = 0;
// if(acc_0.time < acc_1.time && acc_1.time < acc_2.time){
  position = acc_0.data_x;//*(acc_2.time - acc_0.time)/6;
// }
// else if(acc_1.time < acc_2.time && acc_2.time < acc_0.time){
//   position = (acc_1.data_x + 4*acc_2.data_x + acc_0.data_x)*(acc_0.time - acc_1.time)/6;
// }
// else if(acc_2.time < acc_0.time && acc_0.time < acc_1.time){
//   position = (acc_2.data_x + 4*acc_0.data_x + acc_1.data_x)*(acc_1.time - acc_2.time)/6;
// }

 return position;
}

int getPosition_y(ACCEL acc_0, ACCEL acc_1, ACCEL acc_2){
  int position = 0;
  if(acc_0.time < acc_1.time && acc_1.time < acc_2.time){
    position = (acc_0.data_y + 4*acc_1.data_y + acc_2.data_y)*(acc_2.time - acc_0.time)/6;
  }
  else if(acc_1.time < acc_2.time && acc_2.time < acc_0.time){
    position = (acc_1.data_y + 4*acc_2.data_y + acc_0.data_y)*(acc_0.time - acc_1.time)/6;
  }
  else if(acc_2.time < acc_0.time && acc_0.time < acc_1.time){
    position = (acc_2.data_y + 4*acc_0.data_y + acc_1.data_y)*(acc_1.time - acc_2.time)/6;
  }

 return position;
}

int getPosition_z(ACCEL acc_0, ACCEL acc_1, ACCEL acc_2){
  int position = 0;
  if(acc_0.time < acc_1.time && acc_1.time < acc_2.time){
    position = (acc_0.data_z + 4*acc_1.data_z + acc_2.data_z)*(acc_2.time - acc_0.time)/6;
  }
  else if(acc_1.time < acc_2.time && acc_2.time < acc_0.time){
    position = (acc_1.data_z + 4*acc_2.data_z + acc_0.data_z)*(acc_0.time - acc_1.time)/6;
  }
  else if(acc_2.time < acc_0.time && acc_0.time < acc_1.time){
    position = (acc_2.data_z + 4*acc_0.data_z + acc_1.data_z)*(acc_1.time - acc_2.time)/6;
  }

 return position;
}
