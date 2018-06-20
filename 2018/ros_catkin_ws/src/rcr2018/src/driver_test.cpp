#include <iostream>
#include <pigpio.h>
#include <wiringPi.h>
#include <ros/ros.h>
#include <rcr2018/AngVel.h>

const double kp {0.090}; //比例ゲインを決定
const double ki {0.0090}; //積分ゲインを決定

const int PWMPIN_D {18}; //PWMピンのピン配置を18番ピンに
const int DIRPIN {23}; //DIRピンのピン配置を23番ピンに
const int frequency {500}; //DCモータの周波数
const double target_value {150.0}; //目標角速度

double ang_vel {0.0}; //出力角度の初期化
double input_value_pre {0.0}; //一つ前時点での入力
double output_value_pre {0.0}; //一つ前時点での出力角速度
double input_pwm_value_pre {0.0}; //一つ前時点での入力PWM信号
double dev_tar_out {0.0}; //現時点での目標値と出力値の差の初期化
double dev_tar_out_pre {0.0}; //一つ前時点での目標値と出力値の差の初期化

bool is_finish {false}; //終了判定

const int START_SW_PIN {25};

void encodermsgCallback(const rcr2018::AngVel::ConstPtr& msg)
{
  if (!is_finish)
  {
    std::cout << "Duty:" << input_pwm_value_pre << " " << "Dcm:" << output_value_pre << std::endl; //前時点での入力PWM信号と出力角速度の表示

    double output_value {msg->ang_vel}; //ロータリーエンコーダの出力値

    dev_tar_out = target_value - output_value; //目標角速度と出力角度の差分

    double control_value {(kp * (dev_tar_out - dev_tar_out_pre)) + (ki * dev_tar_out)}; //PI制御器による入力値の決定
    
    double input_value {input_value_pre + control_value};

    double input_pwm_value {2240.1 * input_value + 29594.0}; //入力PWM信号のデューティ比を決定
    
    if (input_pwm_value > 1000000)
    {
       input_pwm_value = 1000000;
       std::cout << "Divergence" << std::endl;
    }

    gpioWrite(DIRPIN, 0); //DIRピンの出力を決定
    gpioHardwarePWM(PWMPIN_D, frequency, static_cast<int>(input_pwm_value)); //DCモータにPWM信号を入力

    input_value_pre = input_value;
    input_pwm_value_pre = input_pwm_value; //現時点での入力PWM信号を保存
    output_value_pre = output_value; //現時点での出力角速度を保存
    dev_tar_out_pre = dev_tar_out; //次の時点のため、現時点での値を保存
  }

}

int main(int argc, char** argv)
{
  if (gpioInitialise() < 0) //pigpioの初期化
  {
    std::cout << "error" << std::endl;
    return -1;
  }

  int pin_value {0};

  if (wiringPiSetupGpio() == -1)
  {
     std::cout << "ERROR" << std::endl;
     return -1;
  }

  pinMode(START_SW_PIN, INPUT);

  gpioSetMode(PWMPIN_D, PI_OUTPUT); //PWMピンのセット
  gpioSetMode(DIRPIN, PI_OUTPUT); //DIRピンのセット

  ros::init(argc, argv, "driver"); //ノード名の初期化

  ros::NodeHandle nh; //ノードハンドル宣言

  pin_value = digitalRead(START_SW_PIN);

  while(pin_value != 0)
  {
    pin_value = digitalRead(START_SW_PIN);
  }

  ros::Subscriber encoder_sub = nh.subscribe("ang_vel", 1, encodermsgCallback);

  ros::spin();

  return 0;
}
