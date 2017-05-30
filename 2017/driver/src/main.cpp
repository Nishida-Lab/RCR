#include <driver/driver.hpp>


int main(int argc, char** argv)
{
  robocar::differential_driver driver {std::pair<int,int> {35, 38}, std::pair<int,int> {37, 40}};

  return 0;
}
