#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include <raspicam/raspicam.h>

#include <camera/version.hpp>


int main(int argc, char** argv)
{
  raspicam::RaspiCam camera {};

  if (!camera.open())
  {
    std::cerr << "\e[0;31m" << "[debug] failed to open camera" << "\e[0;37m\n";
    std::exit(EXIT_FAILURE);
  }

  std::cout << "[debug] waiting for camera stabilize...\n";
  std::this_thread::sleep_for(std::chrono::seconds(3));

  std::cout << "[debug] start image capture\n";
  camera.grab();

  std::cout << "[debug] memory allocationg\n";
  unsigned char* data {new unsigned char [camera.getImageTypeSize(raspicam::RASPICAM_FORMAT_RGB)]};

  std::cout << "[debug] extract image in RGB format\n";
  camera.retrieve(data, raspicam::RASPICAM_FORMAT_RGB);

  std::string file_name {"test_image.ppm"};
  std::ofstream ofs {file_name, std::ios::binary};

  std::cout << "[debug] write image\n";
  ofs << "P6\n" << camera.getWidth() << " " << camera.getHeight() << " 255\n";
  ofs.write(reinterpret_cast<char*>(data), camera.getImageTypeSize(raspicam::RASPICAM_FORMAT_RGB));

  std::cout << "[debug] image saved: " << file_name << std::endl;

  delete data;

  return 0;
}

