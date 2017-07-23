#include<./Multiplexer.h>
 
#define GRAVITY 9.80665 //set gravity constant
#define SENSITIVITY 1000


DATA acc_0,acc_1,acc_2,vel_0,vel_1,vel_2,pos; //set struct



void getAcc(int num, unsigned long time, double x, double offset_x, double y, double offset_y, double z, double offset_z){
  switch(num){ //"num" is timing
  case 0:
    acc_0.time = time; //get time
    //重力加速度[m/s^2]*(電圧[mV]-オフセット電圧[mV])/感度[mV/g] = 加速度[m/s^2]
    acc_0.data_x = GRAVITY*(x * 4.9 - offset_x)/SENSITIVITY; //caliculate accelaration 
    acc_0.data_y = GRAVITY*(y * 4.9 - offset_y)/SENSITIVITY; 
    acc_0.data_z = GRAVITY*(z * 4.9 - offset_z)/SENSITIVITY;

    acc_0.data_x = (acc_0.data_x + acc_1.data_x + acc_2.data_x)/3;
    acc_0.data_y = (acc_0.data_y + acc_1.data_y + acc_2.data_y)/3;
    acc_0.data_z = (acc_0.data_z + acc_1.data_z + acc_2.data_z)/3;

    if(abs(acc_0.data_x) < 0.25) acc_0.data_x = 0;  //cut low value
    if(abs(acc_0.data_y) < 0.25) acc_0.data_y = 0;
    if(abs(acc_0.data_z) < 0.25) acc_0.data_z = 0;
    break;
    
  case 1:
    acc_1.time = time;
    acc_1.data_x = GRAVITY*(x * 4.9 - offset_x)/SENSITIVITY; 
    acc_1.data_y = GRAVITY*(y * 4.9 - offset_y)/SENSITIVITY; 
    acc_1.data_z = GRAVITY*(z * 4.9 - offset_z)/SENSITIVITY;

    acc_1.data_x = (acc_0.data_x + acc_1.data_x + acc_2.data_x)/3;
    acc_1.data_y = (acc_0.data_y + acc_1.data_y + acc_2.data_y)/3;
    acc_1.data_z = (acc_0.data_z + acc_1.data_z + acc_2.data_z)/3;
    
    if(abs(acc_1.data_x) < 0.25) acc_1.data_x = 0;
    if(abs(acc_1.data_y) < 0.25) acc_1.data_y = 0;
    if(abs(acc_1.data_z) < 0.25) acc_1.data_z = 0;
    break;

  case 2:
    acc_2.time = time;
    acc_2.data_x = GRAVITY*(x * 4.9 - offset_x)/SENSITIVITY;
    acc_2.data_y = GRAVITY*(y * 4.9 - offset_y)/SENSITIVITY; 
    acc_2.data_z = GRAVITY*(z * 4.9 - offset_z)/SENSITIVITY;

    acc_2.data_x = (acc_0.data_x + acc_1.data_x + acc_2.data_x)/3;
    acc_2.data_y = (acc_0.data_y + acc_1.data_y + acc_2.data_y)/3;
    acc_2.data_z = (acc_0.data_z + acc_1.data_z + acc_2.data_z)/3;
    
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


double buff_vel_x[3] = {0,0,0},buff_vel_y[3] = {0,0,0},buff_vel_z[3] = {0,0,0};
int count_vel = 0;
void getVel(int timing){
  double new_vel_x,new_vel_y,new_vel_z;
  double param = 0.95; //rc filter rapametor
  switch(timing){
  case 0:
  vel_0.time = acc_1.time;
  new_vel_x = getIntegral(vel_0.time, acc_0.data_x, vel_1.time, acc_1.data_x, vel_2.time, acc_2.data_x)/(1000*1000);
  new_vel_y = getIntegral(vel_0.time, acc_0.data_y, vel_1.time, acc_1.data_y, vel_2.time, acc_2.data_y)/(1000*1000);
  new_vel_z = getIntegral(vel_0.time, acc_0.data_z, vel_1.time, acc_1.data_z, vel_2.time, acc_2.data_z)/(1000*1000);
  buff_vel_x[0] = param * buff_vel_x[2] + (1-param) * new_vel_x; //rc filter						
  buff_vel_y[0] = param * buff_vel_y[2] + (1-param) * new_vel_y;						
  buff_vel_z[0] = param * buff_vel_z[2] + (1-param) * new_vel_z;
  buff_vel_x[0] = new_vel_x - buff_vel_x[0]; //High pass filter
  buff_vel_y[0] = new_vel_y - buff_vel_y[0]; 
  buff_vel_z[0] = new_vel_z - buff_vel_z[0];
  if(count_vel > 300){
    vel_0.data_x = vel_2.data_x + buff_vel_x[0]; //accumulation velocity
    vel_0.data_y = vel_2.data_y + buff_vel_y[0];
    vel_0.data_z = vel_2.data_z + buff_vel_z[0];
  }
  count_vel++;
 break;														
														
  case 1:													
  vel_1.time = acc_2.time;											
  new_vel_x = getIntegral(vel_0.time, acc_0.data_x, vel_1.time, acc_1.data_x, vel_2.time, acc_2.data_x)/(1000*1000);
  new_vel_y = getIntegral(vel_0.time, acc_0.data_y, vel_1.time, acc_1.data_y, vel_2.time, acc_2.data_y)/(1000*1000);
  new_vel_z = getIntegral(vel_0.time, acc_0.data_z, vel_1.time, acc_1.data_z, vel_2.time, acc_2.data_z)/(1000*1000);
  buff_vel_x[1] = param * buff_vel_x[0] + (1-param) * new_vel_x;						
  buff_vel_y[1] = param * buff_vel_y[0] + (1-param) * new_vel_y;						
  buff_vel_z[1] = param * buff_vel_z[0] + (1-param) * new_vel_z;
  buff_vel_x[1] = new_vel_x - buff_vel_x[1]; 
  buff_vel_y[1] = new_vel_y - buff_vel_y[1]; 
  buff_vel_z[1] = new_vel_z - buff_vel_z[1];
  if(count_vel > 300){
    vel_1.data_x = vel_0.data_x + buff_vel_x[1];
    vel_1.data_y = vel_0.data_y + buff_vel_y[1];
    vel_1.data_z = vel_0.data_z + buff_vel_z[1];
  }
  count_vel++;
  break;													
														
  case 2:													
  vel_2.time = acc_0.time;											
  new_vel_x = getIntegral(vel_0.time, acc_0.data_x, vel_1.time, acc_1.data_x, vel_2.time, acc_2.data_x)/(1000*1000);
  new_vel_y = getIntegral(vel_0.time, acc_0.data_y, vel_1.time, acc_1.data_y, vel_2.time, acc_2.data_y)/(1000*1000);
  new_vel_z = getIntegral(vel_0.time, acc_0.data_z, vel_1.time, acc_1.data_z, vel_2.time, acc_2.data_z)/(1000*1000);
  buff_vel_x[2] = param * buff_vel_x[1] + (1-param) * new_vel_x;						
  buff_vel_y[2] = param * buff_vel_y[1] + (1-param) * new_vel_y;						
  buff_vel_z[2] = param * buff_vel_z[1] + (1-param) * new_vel_z;
  buff_vel_x[2] = new_vel_x - buff_vel_x[2]; 
  buff_vel_y[2] = new_vel_y - buff_vel_y[2]; 
  buff_vel_z[2] = new_vel_z - buff_vel_z[2];
  if(count_vel > 300){
    vel_2.data_x = vel_1.data_x + buff_vel_x[2];
    vel_2.data_y = vel_1.data_y + buff_vel_y[2];
    vel_2.data_z = vel_1.data_z + buff_vel_z[2];
  }
  count_vel++;
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
    pos.data_x += pos_x/100; // "/100" is arbitary value 
    pos.data_y += pos_y/100; 
    pos.data_z += pos_z/100;    
  }
  count_pos++;
}


