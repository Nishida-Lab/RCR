#include <iostream>
#include <unistd.h>
#include <cstdio>
#include <sstream>
#include "main.hpp"


using namespace std;

int main()
{
  // --------------------------------
  // Camera Section 
  // (It has infinity loop)
  // --------------------------------
  // RaspiCamera camera(320,240,0,0,0);
  // while(cvWaitKey(1) != 27){
  //   camera.img = raspiCamCvQueryFrame(camera.cap);
  //   cvShowImage("RasPi Camera", camera.img);
  // }
  // --------------------------------

  Motor motor;  
  //FILE *fp;
  //fp = fopen("/dev/servoblaster", "w");

  // motor.Drive(1.33, -0.410);
  // cout << "first" << endl;
  // //fprintf(fp, "0=40%");
  // sleep(5);
  cout << "second" << endl;
  motor.Drive(1.0,1.0);
  sleep(5);
  //fprintf(fp, "0=52.5%");
  // sleep(1);
  // motor.Drive(-1.00, 0.410);
  // cout << "third" << endl;
  // //fprintf(fp, "0=65%");
  // sleep(5);

  //fclose(fp);
  return 0 ;
}
