#include <iostream>

#include <main_camera/version.hpp>

int main(int argc, char** argv)
{
  std::cout << "[debug] camera program for robocar race 2017\n"
            << "        version: " << project_version.data() << std::endl;

  return 0;
}

