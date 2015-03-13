#include <iostream>
#include <unistd.h>
#include <cstdio>
#include <exception>
#include <boost/thread.hpp>
#include "main.hpp"

using namespace std;

void Enc_thread(void){
  Encoder enc;
  long rear_cnt, front_cnt;

  while(1){
    enc.GetCnt(front_cnt, rear_cnt);
    cout << __PRETTY_FUNCTION__ << endl;
    cout << "encoder" << endl;
    cout << front_cnt << "\t" << rear_cnt << endl;
  }
}
 
int main()
{
  try {
    Motor motor;  

    boost::thread th1(Enc_thread);
    th1.detach();
    
    for(float i=1.0; i>=-1.0; i-=0.1){
      motor.Drive(1.0,i);
      usleep(100000);
      cout << "main" << endl;
    }
]
  }catch(exception &e){
    cerr << e.what() << endl;
    cerr << "Stop the Program..." << endl;
    exit(1);
  }

  return 0 ;
}
