#include<./Multiplexer.h>

#define OFFSET_X 1642
#define OFFSET_Y 1642
#define OFFSET_Z 1230
#define GRAVITY 9.80665

DATA acc_0,acc_1,acc_2,vel_0,vel_1,vel_2,pos;



void getAcc(int num, double x, double y, double z){
  switch(num){
    case 0:
    acc_0.time = micros();
    //重力加速度[m/s^2]*(電圧[mV]-オフセット電圧[mV])/感度[mV/g] = 加速度[m/s^2]
    acc_0.data_x = 9.8*(x * 4.9 - OFFSET_X)/1000; 
    acc_0.data_y = 9.8*(y * 4.9 - OFFSET_Y)/1000; 
    acc_0.data_z = 9.8*(z * 4.9 - OFFSET_Z)/1000;
    acc_0.data_x = (acc_0.data_x + acc_1.data_x + acc_2.data_x)/3;
    acc_0.data_y = (acc_0.data_y + acc_1.data_y + acc_2.data_y)/3;
    acc_0.data_z = (acc_0.data_z + acc_1.data_z + acc_2.data_z)/3;
    break;
 
  case 1:
    acc_1.time = micros();
    acc_1.data_x = 9.8*(x * 4.9 - OFFSET_X)/1000; 
    acc_1.data_y = 9.8*(y * 4.9 - OFFSET_Y)/1000; 
    acc_1.data_z = 9.8*(z * 4.9 - OFFSET_Z)/1000;
    acc_1.data_x = (acc_0.data_x + acc_1.data_x + acc_2.data_x)/3;
    acc_1.data_y = (acc_0.data_y + acc_1.data_y + acc_2.data_y)/3;
    acc_1.data_z = (acc_0.data_z + acc_1.data_z + acc_2.data_z)/3;
    break;

  case 2:
    acc_2.time = micros();
    acc_2.data_x = 9.8*(x * 4.9 - OFFSET_X)/1000;
    acc_2.data_y = 9.8*(y * 4.9 - OFFSET_Y)/1000; 
    acc_2.data_z = 9.8*(z * 4.9 - OFFSET_Z)/1000;
    acc_2.data_x = (acc_0.data_x + acc_1.data_x + acc_2.data_x)/3;
    acc_2.data_y = (acc_0.data_y + acc_1.data_y + acc_2.data_y)/3;
    acc_2.data_z = (acc_0.data_z + acc_1.data_z + acc_2.data_z)/3;
    break;
  }
}

double getIntegral(unsigned long time_0, double data_0, unsigned long time_1, double data_1, unsigned long time_2, double data_2){
  double answer = 0;
  if(time_0 < time_1 && time_1 < time_2){
    answer = (data_0 + 4*data_1 + data_2)*(time_2 - time_0)/6;
  }
  else if(time_1 < time_2 && time_2 < time_0){
    answer = (data_1 + 4*data_2 + data_0)*(time_0 - time_1)/6;
  }
  else if(time_2 < time_0 && time_0 < time_1){
    answer = (data_2 + 4*data_0 + data_1)*(time_1 - time_2)/6;
  }

  return answer;
}


void getVel(int timing){
  acc_0.data_z -= GRAVITY;
  acc_1.data_z -= GRAVITY;
  acc_2.data_z -= GRAVITY;
  
  switch(timing){
  case 0:
  vel_0.time = acc_1.time;
  vel_0.data_x = getIntegral(vel_0.time, acc_0.data_x, vel_1.time, acc_1.data_x, vel_2.time, acc_2.data_x)/(1000*1000);
  vel_0.data_y = getIntegral(vel_0.time, acc_0.data_y, vel_1.time, acc_1.data_y, vel_2.time, acc_2.data_y)/(1000*1000);
  vel_0.data_z = getIntegral(vel_0.time, acc_0.data_z, vel_1.time, acc_1.data_z, vel_2.time, acc_2.data_z)/(1000*1000);
  break;										 
											 
  case 1:										 
  vel_1.time = acc_2.time;								 
  vel_1.data_x = getIntegral(vel_0.time, acc_0.data_x, vel_1.time, acc_1.data_x, vel_2.time, acc_2.data_x)/(1000*1000);
  vel_1.data_y = getIntegral(vel_0.time, acc_0.data_y, vel_1.time, acc_1.data_y, vel_2.time, acc_2.data_y)/(1000*1000);
  vel_1.data_z = getIntegral(vel_0.time, acc_0.data_z, vel_1.time, acc_1.data_z, vel_2.time, acc_2.data_z)/(1000*1000);
  break;										     
											     
  case 2:										     
  vel_2.time = acc_0.time;								     
  vel_2.data_x = getIntegral(vel_0.time, acc_0.data_x, vel_1.time, acc_1.data_x, vel_2.time, acc_2.data_x)/(1000*1000);
  vel_2.data_y = getIntegral(vel_0.time, acc_0.data_y, vel_1.time, acc_1.data_y, vel_2.time, acc_2.data_y)/(1000*1000);
  vel_2.data_z = getIntegral(vel_0.time, acc_0.data_z, vel_1.time, acc_1.data_z, vel_2.time, acc_2.data_z)/(1000*1000);
  break;
  }
}

void getPos(){
  pos.data_x = getIntegral(vel_0.time, vel_0.data_x, vel_1.time, vel_1.data_x, vel_2.time, vel_2.data_x)/(1000*1000);
  pos.data_y = getIntegral(vel_0.time, vel_0.data_y, vel_1.time, vel_1.data_y, vel_2.time, vel_2.data_y)/(1000*1000);
  pos.data_z = getIntegral(vel_0.time, vel_0.data_z, vel_1.time, vel_1.data_z, vel_2.time, vel_2.data_z)/(1000*1000);
}
