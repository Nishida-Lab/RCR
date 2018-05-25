#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <vl53l0x.h>


//各センサのシャットダウンピン番号を定義
const int center_shutdown = 16;
const int left_shutdown = 20;
const int right_shut_down = 21;

int main(int argc, char** argv)
{
  //Tofセンサの初期化
  int object_number = 0;
  int address = 0x29;
  int TCA9548A_Device = 255;
  int TCA9548A_Address = 0;

  startRanging(object_number, VL53L0X_BETTER_ACCURACY_MODE, address, TCA9548A_Device, TCA9548A_Address);
  std::cout << "start ranging" << '\n';
  stopRanging(object_number);

  return 0;
  
}
