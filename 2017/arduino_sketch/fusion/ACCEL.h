#include<./Multiplexer.h>

#define OFFSET_X 1801.7
#define OFFSET_Y 1856

struct DATA{
  unsigned long time;
  double data_x;
  double data_y;
  double data_z;
};


DATA acc_0,acc_1,acc_2,vel_0,vel_1,vel_2;

void getAcc(int num, double x, double y, double z){
  switch(num){
    case 0:
    acc_0.time = micros();
    //重力加速度[m/s^2]*(電圧[mV]-オフセット電圧[mV])/感度[mV/g] = 加速度[m/s^2]
    acc_0.data_x = 9.8*(x * 4.9 - OFFSET_X)/660; 
    acc_0.data_y = 9.8*(y * 4.9 - OFFSET_Y)/660; 
    acc_0.data_z = 9.8*(z * 4.9)/660; break;
 
  case 1:
    acc_1.time = micros();
    acc_1.data_x = 9.8*(x * 4.9 - OFFSET_X)/660; 
    acc_1.data_y = 9.8*(y * 4.9 - OFFSET_Y)/660; 
    acc_1.data_z = 9.8*(z * 4.9 - 1650)/660; break;

  case 2:
    acc_2.time = micros();
    acc_2.data_x = 9.8*(x * 4.9-OFFSET_X)/660;
    acc_2.data_y = 9.8*(y * 4.9-OFFSET_Y)/660; 
    acc_2.data_z = 9.8*(z * 4.9-1650)/660; break;
  }
}

int getVelocity(unsigned long time_0, int acc_0, unsigned long time_1, int acc_1, unsigned long time_2, int acc_2){
  int velocity = 0;
  if(time_0 < time_1 && time_1 < time_2){
    velocity = (acc_0 + 4*acc_1 + acc_2)*(time_2 - time_0)/6;
  }
  else if(time_1 < time_2 && time_2 < time_0){
    velocity = (acc_1 + 4*acc_2 + acc_0)*(time_0 - time_1)/6;
  }
  else if(time_2 < time_0 && time_0 < time_1){
    velocity = (acc_2 + 4*acc_0 + acc_1)*(time_1 - time_2)/6;
  }

  return velocity;
}


int getPosition(unsigned long time_0, int vel_0, unsigned long time_1, int vel_1, unsigned long time_2, int vel_2){
  int position = 0;
  if(time_0 < time_1 && time_1 < time_2){
    position = (vel_0 + 4*vel_1 + vel_2)*(time_2 - time_0)/6;
  }
  else if(time_1 < time_2 && time_2 < time_0){
    position = (vel_1 + 4*vel_2 + vel_0)*(time_0 - time_1)/6;
  }
  else if(time_2 < time_0 && time_0 < time_1){
    position = (vel_2 + 4*vel_0 + vel_1)*(time_1 - time_2)/6;
  }

  return position;
}
