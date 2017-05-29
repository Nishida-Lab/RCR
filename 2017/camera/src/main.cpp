#include <camera/camera.hpp>


int main(int argc, char** argv)
{
  robocar::camera camera {1280, 960};

  camera.debug();
  camera.find();

  return 0;
}

