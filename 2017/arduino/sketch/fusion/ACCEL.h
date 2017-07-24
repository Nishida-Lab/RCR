#include<./Multiplexer.h>
 
#define GRAVITY 9.80665 //set gravity constant
#define SENSITIVITY 660
#define ACC_OFFSET_X 1662
#define ACC_OFFSET_Y 1606
#define ACC_OFFSET_Z 1586.65


DATA acc_0,acc_1,acc_2,vel_0,vel_1,vel_2,pos; //set struct



void getAcc(int num, unsigned long time, double x, double y, double z, double offset_x, double offset_y, double offset_z){
  int param = 0.97;
  switch(num){ //"num" is timing
  case 0:
    acc_0.time = time; //get time
    //重力加速度[m/s^2]*(電圧[mV]-オフセット電圧[mV])/感度[mV/g] = 加速度[m/s^2]
    acc_0.data_x = GRAVITY*(x * 4.9 - offset_x)/SENSITIVITY; //caliculate accelaration 
    acc_0.data_y = GRAVITY*(y * 4.9 - offset_y)/SENSITIVITY; 
    acc_0.data_z = GRAVITY*(z * 4.9 - offset_z)/SENSITIVITY;
    
    if(abs(acc_0.data_x) < 0.25) acc_0.data_x = 0;  //cut low value
    if(abs(acc_0.data_y) < 0.25) acc_0.data_y = 0;
    if(abs(acc_0.data_z-GRAVITY) < 0.25) acc_0.data_z = GRAVITY;
    break;
    
  case 1:
    acc_1.time = time;
    acc_1.data_x = GRAVITY*(x * 4.9 - offset_x)/SENSITIVITY; 
    acc_1.data_y = GRAVITY*(y * 4.9 - offset_y)/SENSITIVITY; 
    acc_1.data_z = GRAVITY*(z * 4.9 - offset_z)/SENSITIVITY;
    
    if(abs(acc_1.data_x) < 0.25) acc_1.data_x = 0;
    if(abs(acc_1.data_y) < 0.25) acc_1.data_y = 0;
    if(abs(acc_1.data_z-GRAVITY) < 0.25) acc_1.data_z = GRAVITY;
    break;

  case 2:
    acc_2.time = time;
    acc_2.data_x = GRAVITY*(x * 4.9 - offset_x)/SENSITIVITY;
    acc_2.data_y = GRAVITY*(y * 4.9 - offset_y)/SENSITIVITY; 
    acc_2.data_z = GRAVITY*(z * 4.9 - offset_z)/SENSITIVITY;

    if(abs(acc_2.data_x) < 0.25) acc_2.data_x = 0;
    if(abs(acc_2.data_y) < 0.25) acc_2.data_y = 0;
    if(abs(acc_2.data_z-GRAVITY) < 0.25) acc_2.data_z = GRAVITY;
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

  buff_vel_x[0] = (buff_vel_x[1] + buff_vel_x[2] + new_vel_x)/3; //rc filter						
  buff_vel_y[0] = (buff_vel_y[1] + buff_vel_y[2] + new_vel_y)/3;						

  buff_vel_x[0] = new_vel_x - buff_vel_x[0]; //High pass filter
  buff_vel_y[0] = new_vel_y - buff_vel_y[0]; 

  vel_0.data_x = buff_vel_x[0];// + vel_2.data_x; //accumulation velocity
  vel_0.data_y = buff_vel_y[0];// + vel_2.data_y;
 break;														
														
  case 1:													
  vel_1.time = acc_2.time;											
  new_vel_x = getIntegral(vel_0.time, acc_0.data_x, vel_1.time, acc_1.data_x, vel_2.time, acc_2.data_x)/(1000*1000);
  new_vel_y = getIntegral(vel_0.time, acc_0.data_y, vel_1.time, acc_1.data_y, vel_2.time, acc_2.data_y)/(1000*1000);

  buff_vel_x[1] = (buff_vel_x[0] + buff_vel_x[2] + new_vel_x)/3;						
  buff_vel_y[1] = (buff_vel_y[0] + buff_vel_y[2] + new_vel_y)/3;						

  buff_vel_x[1] = new_vel_x - buff_vel_x[1]; 
  buff_vel_y[1] = new_vel_y - buff_vel_y[1]; 

  vel_1.data_x = buff_vel_x[1];// + vel_0.data_x;
  vel_1.data_y = buff_vel_y[1];// + vel_0.data_y;
  break;													
														
  case 2:													
  vel_2.time = acc_0.time;											
  new_vel_x = getIntegral(vel_0.time, acc_0.data_x, vel_1.time, acc_1.data_x, vel_2.time, acc_2.data_x)/(1000*1000);
  new_vel_y = getIntegral(vel_0.time, acc_0.data_y, vel_1.time, acc_1.data_y, vel_2.time, acc_2.data_y)/(1000*1000);

  buff_vel_x[2] = (buff_vel_x[1] + buff_vel_x[0] + new_vel_x)/3;						
  buff_vel_y[2] = (buff_vel_y[1] + buff_vel_y[0] + new_vel_y)/3;						

  buff_vel_x[2] = new_vel_x - buff_vel_x[2]; 
  buff_vel_y[2] = new_vel_y - buff_vel_y[2]; 

  vel_2.data_x = buff_vel_x[2];// + vel_1.data_x;
  vel_2.data_y = buff_vel_y[2];// + vel_1.data_y;
  break;
  }
}

int count_pos = 0;
void getPos(int timing){
  double new_pos_x, new_pos_y, new_pos_z;
  double pos_x[3], pos_y[3], pos_z[3];
  double param = 0.95;

  switch(timing){
  case 0:
    new_pos_x = getIntegral(vel_0.time, vel_0.data_x, vel_1.time, vel_1.data_x, vel_2.time, vel_2.data_x)/(1000*1000);
    new_pos_y = getIntegral(vel_0.time, vel_0.data_y, vel_1.time, vel_1.data_y, vel_2.time, vel_2.data_y)/(1000*1000);

    pos_x[0] = (new_pos_x + pos_x[1] + pos_x[2])/3;
    pos_y[0] = (new_pos_y + pos_y[1] + pos_y[2])/3;

    pos_x[0] = new_pos_x - pos_x[0]; 
    pos_y[0] = new_pos_y - pos_y[0]; 

    pos.data_x += pos_x[0]/100; // "/100" is arbitary value 
    pos.data_y += pos_y[0]/100; 
    break;
 
  case 1:
    new_pos_x = getIntegral(vel_0.time, vel_0.data_x, vel_1.time, vel_1.data_x, vel_2.time, vel_2.data_x)/(1000*1000);
    new_pos_y = getIntegral(vel_0.time, vel_0.data_y, vel_1.time, vel_1.data_y, vel_2.time, vel_2.data_y)/(1000*1000);

    pos_x[1] = (new_pos_x + pos_x[0] + pos_x[2])/3;
    pos_y[1] = (new_pos_y + pos_y[0] + pos_y[2])/3;

    pos_x[1] = new_pos_x - pos_x[1]; 
    pos_y[1] = new_pos_y - pos_y[1]; 

    pos.data_x += pos_x[1]/100; // "/100" is arbitary value 
    pos.data_y += pos_y[1]/100; 
    break;
 
  case 2:
    new_pos_x = getIntegral(vel_0.time, vel_0.data_x, vel_1.time, vel_1.data_x, vel_2.time, vel_2.data_x)/(1000*1000);
    new_pos_y = getIntegral(vel_0.time, vel_0.data_y, vel_1.time, vel_1.data_y, vel_2.time, vel_2.data_y)/(1000*1000);

    pos_x[2] = (new_pos_x + pos_x[1] + pos_x[0])/3;
    pos_y[2] = (new_pos_y + pos_y[1] + pos_y[0])/3;

    pos_x[2] = new_pos_x - pos_x[2]; 
    pos_y[2] = new_pos_y - pos_y[2]; 

    pos.data_x += pos_x[2]/100; // "/100" is arbitary value 
    pos.data_y += pos_y[2]/100; 
    break;    
  }
}


