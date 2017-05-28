#include <camera/camera.hpp>


int main(int argc, char** argv)
{
  robocar::camera camera {1280, 960};

  // camera.read();
  // camera.write("hoge.jpg");

  camera.debug();

  return 0;
}

