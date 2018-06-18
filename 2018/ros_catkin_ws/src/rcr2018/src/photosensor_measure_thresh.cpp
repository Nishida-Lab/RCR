#include <iostream>
#include <wiringPi.h>
#include <unistd.h>

#define GPIO_PIN 4

void sensor_charge()
{
  pinMode(GPIO_PIN, OUTPUT);
  digitalWrite(GPIO_PIN, 1);
  usleep(10);
  pinMode(GPIO_PIN, INPUT);
}

int main(int argc, char** argv)
{
  if(wiringPiSetupGpio() == -1)
  {
    std::cout << "ERROR:Can't setup GPIO." << std::endl;
    return -1;
  }

  pullUpDnControl(GPIO_PIN, PUD_UP);

  unsigned int black_pulse_width {0};
  unsigned int white_pulse_width {0};
  unsigned long sum {0};
  const int loop_count {100};
  unsigned int thresh {0};

  // 黒色測定
  std::cout << "PRESS KEY TO MEASURE BLACK PULSE WIDTH..." << std::endl;
  getchar();

  for (int i = 0; i < loop_count; i++)
  {
      sensor_charge();
      while (digitalRead(GPIO_PIN) == 1)
      {
        ++black_pulse_width;
      }

      sum += black_pulse_width;
      black_pulse_width = 0;

  }
  black_pulse_width = static_cast<unsigned int>(sum / loop_count);

  // 白色測定
  std::cout << "PRESS KEY TO MEASURE WHITE PULSE WIDTH..." << std::endl;
  getchar();
  sum = 0;

  for (int i = 0; i < loop_count; i++)
  {
      sensor_charge();
      while (digitalRead(GPIO_PIN) == 1)
      {
        ++white_pulse_width;
      }

      sum += white_pulse_width;
      white_pulse_width = 0;

  }
  white_pulse_width = static_cast<unsigned int>(sum / loop_count);

  //しきい値を計算し表示
  thresh = static_cast<unsigned int>((black_pulse_width + white_pulse_width) / 2);
  std::cout << "black:" << black_pulse_width << std::endl;
  std::cout << "white:" << white_pulse_width << std::endl;
  std::cout << "---------------------------" << std::endl;
  std::cout << "thresh:" << thresh << std::endl;

  return 0;
}
