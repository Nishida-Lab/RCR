#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include <raspicam/raspicam.h>

#include <main_camera/version.hpp>

int main(int argc, char** argv)
{
  raspicam::RaspiCam camera {};

  if (!camera.open())
  {
    std::cerr << "[debug] failed to open camera\n";
    std::exit(EXIT_FAILURE);
  }

  std::cout << "[debug] waiting for camera stabilize...\n";
  std::this_thread::sleep_for(std::chrono::seconds(3));

  return 0;
}

