#include "camera.hpp"
#include <iostream>

using namespace std;

RaspiCamera::RaspiCamera() :
  width(320),
  height(240),
  bitrate(0),
  framerate(0),
  monochrome(0)
{
  // Get dynamic memory for configuration of the camera
  try{
    config = new RASPIVID_CONFIG [sizeof(RASPIVID_CONFIG)];
    if(!config)
      throw 1;
    else
      throw 0;
  }
  catch(int E_cord){
    // if faile to get dynamic memory
    if(E_cord){
      cerr << "Failed to get dynamic memory." << endl;
      exit(1);
    }
    // if success to get dynamic memory
    else{
      // Set the configuration 
      config->width = width;
      config->height = height;
      config->bitrate = bitrate;
      config->monochrome = monochrome;
      
      // camera setting
      cap = (RaspiCamCvCapture *)raspiCamCvCreateCameraCapture2(0,config);
      // delete the configuration
      delete [] config;

      // get the memory for Iplimage
      img = raspiCamCvQueryFrame(cap);
    }
  }
}

RaspiCamera::RaspiCamera(int w, int h, int b, int f, int m) :
  width(w),
  height(h),
  bitrate(b),
  framerate(f),
  monochrome(m)
{
  try{
    config = new RASPIVID_CONFIG [sizeof(RASPIVID_CONFIG)];
    if(!config)
      throw 1;
    else
      throw 0;
  }
  catch(int E_cord){
    if(E_cord){
      cerr << "Failed to get dynamic memory." << endl;
      exit(1);
    }
    else{
      config->width = width;
      config->height = height;
      config->bitrate = bitrate;
      config->monochrome = monochrome;

      cap = (RaspiCamCvCapture *)raspiCamCvCreateCameraCapture2(0,config);
      delete [] config;
      img = raspiCamCvQueryFrame(cap);
    }
  }
}

RaspiCamera::~RaspiCamera()
{
  cvDestroyAllWindows();
  raspiCamCvReleaseCapture(&cap);
  cvReleaseImage(&img);
}
