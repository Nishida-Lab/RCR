#include <iostream>
#include <wiringPi.h>
#include <unistd.h>
#include <ros/ros.h>
#include <rcr2018/LineCount.h>

#define GPIO_PIN 10 

void sensor_charge()
{
  pinMode(GPIO_PIN, OUTPUT);
  digitalWrite(GPIO_PIN, 1);
  usleep(10);
  pinMode(GPIO_PIN, INPUT);
}

// 色判別のしきい値
const unsigned int thresh_color = 3000;
// タイムアウト
const unsigned int timeout = 5000;
// コントロールライン検出とみなすのに必要な連続白色検出回数
const unsigned int thresh_read_count = 1;
// コントロールライン検出後のスリープ時間[s]
const unsigned int sleep_sec = 5;
// スタートスイッチピン番号
const int START_SW_PIN {25};
// ループカウント数
const int loop_count {100};

int main(int argc, char** argv)
{

  int pin_value {0};

  if(wiringPiSetupGpio() == -1)
  {
    std::cout << "ERROR:Can't setup GPIO." << std::endl;
    return -1;
  }

  pinMode(START_SW_PIN, INPUT);

  pullUpDnControl(GPIO_PIN, PUD_UP);

  pin_value = digitalRead(START_SW_PIN);
  while(pin_value != 0)
  {
    pin_value = digitalRead(START_SW_PIN);
  }

  unsigned int pulse_width {0};
  unsigned int read_count {0};
  unsigned long sum {0};

  //ROS周りのセットアップ
  ros::init(argc, argv, "photosensor");
  ros::NodeHandle nh;
  ros::Publisher line_count_pub = nh.advertise<rcr2018::LineCount>("line_count", 1);
  rcr2018::LineCount msg;
  msg.count = 0;

  while (ros::ok())
  {
    pulse_width = 0;
    sum = 0;

    for (int i = 0; i < loop_count; i++)
    {
      sensor_charge();

      while (digitalRead(GPIO_PIN) == 1)
      {
        ++pulse_width;
	if (pulse_width > timeout)
	{
	  break;
	}
      }
      sum += pulse_width;
      pulse_width = 0;
    }

    pulse_width = static_cast<unsigned int>(sum / loop_count);
    // 白色の場合
    if (pulse_width < thresh_color)
    {
      ++read_count;
      std::cout << "white" << std::endl;


      // 既定回以上白を検出した場合コントロールライン通過とみなす
      if (read_count >= thresh_read_count)
      {
        std::cout << "###############detected###############" << std::endl;
        msg.count = msg.count + 1;
        line_count_pub.publish(msg);
        read_count = 0;
        sleep(sleep_sec);
      }

    }
    // 黒色の場合
    else
    {
      std::cout << "black" << std::endl;
      read_count = 0;
    }

  }

  return 0;
}
