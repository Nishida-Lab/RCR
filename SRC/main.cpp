#include <iostream>
#include <unistd.h>
#include <cstdio>
#include <sstream>
#include <exception>
#include "main.hpp"


using namespace std;

int main()
{
  try {
    Motor motor;  
    
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
