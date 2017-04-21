#include <iostream>
#include <unistd.h>
#include <cstdio>
#include <exception>
#include <boost/thread.hpp>
#include <cmath>
#include "main.hpp"

using namespace std;

long rear_cnt, front_cnt;

void Enc_thread(void){
  Encoder enc;

  float tmp;


  while(1){
    enc.GetCnt(front_cnt, rear_cnt);
  }
}
 
int main()
{
  try {
    Motor motor;  
    float dist;
    long local_rear_cnt;
    float speed;
    float angular;
    float rate;

    boost::thread th1(Enc_thread);
    th1.detach();
    
    cout << "main start" << endl;
    rate = 1.0;
    speed = motor.max_speed*rate;//*0.5;
    // for(int i=1;; i++){
    //   if(i%2)
    // 	motor.Drive(speed, 0);
    //   else
    // 	motor.Drive(speed, speed*2.0);
    //   sleep(1);
    // }
    while(dist < 3.14){
      angular *= rate+0.5;
      motor.Drive(speed, speed*2.0);
      local_rear_cnt = rear_cnt;
      local_rear_cnt *= -0.5*rate+1.25;
      dist = ((float)local_rear_cnt / 540.0) * motor.wheel_diameter * M_PI;
    }
    motor.Drive(0,0);
    cout << speed << endl;
    cout << dist << endl;
  }catch(exception &e){
    cerr << e.what() << endl;
    cerr << "Stop the Program..." << endl;
    exit(1);
  }

  return 0 ;
}
