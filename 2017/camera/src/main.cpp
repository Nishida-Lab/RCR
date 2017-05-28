#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <camera/camera.hpp>


#define CONSOLE_DEBUG
// #undef  CONSOLE_DEBUG


int main(int argc, char** argv)
{
  robocar::camera camera {1280, 960};

  // camera.read();
  // camera.write("hoge.jpg");

  camera.debug();

  return 0;
}

