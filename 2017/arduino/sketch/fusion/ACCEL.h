#include<./Multiplexer.h>
 
#define GRAVITY 9.80665

DATA acc_0,acc_1,acc_2,vel_0,vel_1,vel_2,pos;



void getAcc(int num, double x, double offset_x, double y, double offset_y, double z, double offset_z){
  switch(num){
    case 0:
    acc_0.time = micros();
    //重力加速度[m/s^2]*(電圧[mV]-オフセット電圧[mV])/感度[mV/g] = 加速度[m/s^2]
    acc_0.data_x = GRAVITY*(x * 4.9 - offset_x)/1000; 
    acc_0.data_y = GRAVITY*(y * 4.9 - offset_y)/1000; 
    acc_0.data_z = GRAVITY*(z * 4.9 - offset_z)/1000;
    if(abs(acc_0.data_x) < 0.25) acc_0.data_x = 0;
    if(abs(acc_0.data_y) < 0.25) acc_0.data_y = 0;
    if(abs(acc_0.data_z) < 0.25) acc_0.data_z = 0;
    break;
 
  case 1:
    acc_1.time = micros();
    acc_1.data_x = GRAVITY*(x * 4.9 - offset_x)/1000; 
    acc_1.data_y = GRAVITY*(y * 4.9 - offset_y)/1000; 
    acc_1.data_z = GRAVITY*(z * 4.9 - offset_z)/1000;
    if(abs(acc_1.data_x) < 0.25) acc_1.data_x = 0;
    if(abs(acc_1.data_y) < 0.25) acc_1.data_y = 0;
    if(abs(acc_1.data_z) < 0.25) acc_1.data_z = 0;
    break;

  case 2:
    acc_2.time = micros();
    acc_2.data_x = GRAVITY*(x * 4.9 - offset_x)/1000;
    acc_2.data_y = GRAVITY*(y * 4.9 - offset_y)/1000; 
    acc_2.data_z = GRAVITY*(z * 4.9 - offset_z)/1000;
    if(abs(acc_2.data_x) < 0.25) acc_2.data_x = 0;
    if(abs(acc_2.data_y) < 0.25) acc_2.data_y = 0;
    if(abs(acc_2.data_z) < 0.25) acc_2.data_z = 0;
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
  double new_vel_x,new_vel_y,new_vel_z;
  double param = 0.95;
  switch(timing){
  case 0:
  vel_0.time = acc_1.time;
  new_vel_x = getIntegral(vel_0.time, acc_0.data_x, vel_1.time, acc_1.data_x, vel_2.time, acc_2.data_x)/(1000*1000);
  new_vel_y = getIntegral(vel_0.time, acc_0.data_y, vel_1.time, acc_1.data_y, vel_2.time, acc_2.data_y)/(1000*1000);
  new_vel_z = getIntegral(vel_0.time, acc_0.data_z, vel_1.time, acc_1.data_z, vel_2.time, acc_2.data_z)/(1000*1000);
  vel_0.data_x = param * vel_2.data_x + (1-param) * new_vel_x;						
  vel_0.data_y = param * vel_2.data_y + (1-param) * new_vel_y;						
  vel_0.data_z = param * vel_2.data_z + (1-param) * new_vel_z;
  vel_0.data_x = new_vel_x - vel_0.data_x; 
  vel_0.data_y = new_vel_y - vel_0.data_y; 
  vel_0.data_z = new_vel_z - vel_0.data_z;
  vel_0.data_x = (vel_0.data_x + vel_1.data_x + vel_2.data_x )/3; 
  vel_0.data_y = (vel_0.data_y + vel_1.data_y + vel_2.data_y )/3; 
  vel_0.data_z = (vel_0.data_z + vel_1.data_z + vel_2.data_z )/3;  
 break;														
														
  case 1:													
  vel_1.time = acc_2.time;											
  new_vel_x = getIntegral(vel_0.time, acc_0.data_x, vel_1.time, acc_1.data_x, vel_2.time, acc_2.data_x)/(1000*1000);
  new_vel_y = getIntegral(vel_0.time, acc_0.data_y, vel_1.time, acc_1.data_y, vel_2.time, acc_2.data_y)/(1000*1000);
  new_vel_z = getIntegral(vel_0.time, acc_0.data_z, vel_1.time, acc_1.data_z, vel_2.time, acc_2.data_z)/(1000*1000);
  vel_1.data_x = param * vel_0.data_x + (1-param) * new_vel_x;						
  vel_1.data_y = param * vel_0.data_y + (1-param) * new_vel_y;						
  vel_1.data_z = param * vel_0.data_z + (1-param) * new_vel_z;
  vel_1.data_x = new_vel_x - vel_1.data_x; 
  vel_1.data_y = new_vel_y - vel_1.data_y; 
  vel_1.data_z = new_vel_z - vel_1.data_z;
  vel_1.data_x = (vel_0.data_x + vel_1.data_x + vel_2.data_x )/3; 
  vel_1.data_y = (vel_0.data_y + vel_1.data_y + vel_2.data_y )/3; 
  vel_1.data_z = (vel_0.data_z + vel_1.data_z + vel_2.data_z )/3;  
  break;													
														
  case 2:													
  vel_2.time = acc_0.time;											
  new_vel_x = getIntegral(vel_0.time, acc_0.data_x, vel_1.time, acc_1.data_x, vel_2.time, acc_2.data_x)/(1000*1000);
  new_vel_y = getIntegral(vel_0.time, acc_0.data_y, vel_1.time, acc_1.data_y, vel_2.time, acc_2.data_y)/(1000*1000);
  new_vel_z = getIntegral(vel_0.time, acc_0.data_z, vel_1.time, acc_1.data_z, vel_2.time, acc_2.data_z)/(1000*1000);
  vel_2.data_x = param * vel_1.data_x + (1-param) * new_vel_x;
  vel_2.data_y = param * vel_1.data_y + (1-param) * new_vel_y; 
  vel_2.data_z = param * vel_1.data_z + (1-param) * new_vel_z;
  vel_2.data_x = new_vel_x - vel_2.data_x; 
  vel_2.data_y = new_vel_y - vel_2.data_y; 
  vel_2.data_z = new_vel_z - vel_2.data_z;
  vel_2.data_x = (vel_0.data_x + vel_1.data_x + vel_2.data_x )/3; 
  vel_2.data_y = (vel_0.data_y + vel_1.data_y + vel_2.data_y )/3; 
  vel_2.data_z = (vel_0.data_z + vel_1.data_z + vel_2.data_z )/3;  
  break;
  }
}

int count_pos = 0;
void getPos(){
  double new_pos_x, new_pos_y, new_pos_z;
  double pos_x, pos_y, pos_z;
  double param = 0.95;
  new_pos_x = getIntegral(vel_0.time, vel_0.data_x, vel_1.time, vel_1.data_x, vel_2.time, vel_2.data_x)/(1000*1000);
  new_pos_y = getIntegral(vel_0.time, vel_0.data_y, vel_1.time, vel_1.data_y, vel_2.time, vel_2.data_y)/(1000*1000);
  new_pos_z = getIntegral(vel_0.time, vel_0.data_z, vel_1.time, vel_1.data_z, vel_2.time, vel_2.data_z)/(1000*1000);
  pos_x = param * pos_x + (1-param) * new_pos_x; 
  pos_y = param * pos_y + (1-param) * new_pos_y; 
  pos_z = param * pos_z + (1-param) * new_pos_z;  
  pos_x = new_pos_x - pos_x; 
  pos_y = new_pos_y - pos_y; 
  pos_z = new_pos_z - pos_z;

  if(count_pos > 300){
    pos.data_x += pos_x/10; 
    pos.data_y += pos_y/10; 
    pos.data_z += pos_z/10;    
  }
  count_pos++;
}
