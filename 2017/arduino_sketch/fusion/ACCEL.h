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

double rc_filter(double new_data, double last_answer, double param){
  double answer = 0;
  return answer = param * last_answer + (1-param) * new_data;
}


void getVelocity(){
 vel_0.time = acc_1.time;
  vel_0.data_x = getIntegral(acc_0.time, acc_0.data_x, acc_1.time, acc_1.data_x, acc_2.time, acc_2.data_x)/(1000*1000);
  vel_0.data_y = getIntegral(acc_0.time, acc_0.data_y, acc_1.time, acc_1.data_y, acc_2.time, acc_2.data_y)/(1000*1000);
  vel_0.data_z = getIntegral(acc_0.time, acc_0.data_z, acc_1.time, acc_1.data_z, acc_2.time, acc_2.data_z)/(1000*1000);

  vel_1.time = acc_2.time;
  vel_1.data_x = getIntegral(acc_0.time, acc_0.data_x, acc_1.time, acc_1.data_x, acc_2.time, acc_2.data_x)/(1000*1000);
  vel_1.data_y = getIntegral(acc_0.time, acc_0.data_y, acc_1.time, acc_1.data_y, acc_2.time, acc_2.data_y)/(1000*1000);
  vel_1.data_z = getIntegral(acc_0.time, acc_0.data_z, acc_1.time, acc_1.data_z, acc_2.time, acc_2.data_z)/(1000*1000);

  vel_2.time = acc_0.time;
  vel_2.data_x = getIntegral(acc_0.time, acc_0.data_x, acc_1.time, acc_1.data_x, acc_2.time, acc_2.data_x)/(1000*1000);
  vel_2.data_y = getIntegral(acc_0.time, acc_0.data_y, acc_1.time, acc_1.data_y, acc_2.time, acc_2.data_y)/(1000*1000);
  vel_2.data_z = getIntegral(acc_0.time, acc_0.data_z, acc_1.time, acc_1.data_z, acc_2.time, acc_2.data_z)/(1000*1000);
}
