#include <iostream>
#include <unistd.h>
#include <cstdio>
#include <exception>
#include <boost/thread.hpp>
#include "main.hpp"


using namespace std;

void encoder(){
  while(1){
    cout << __PRETTY_FUNCTION__ << endl;
    cout << "encoder" << endl;
  }
}
 
int main()
{
  try {
    Motor motor;  

    boost::thread th1(encoder);
    th1.detach();
    
    for(float i=1.0; i>=-1.0; i-=0.01){
      motor.Drive(1.0,i);
      usleep(100000);
    }

  }catch(exception &e){
    cerr << e.what() << endl;
    cerr << "Stop the Program..." << endl;
    exit(1);
  }

  return 0 ;
}
