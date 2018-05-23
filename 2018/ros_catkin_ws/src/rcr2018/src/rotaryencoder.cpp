#include <iostream>
#include <time.h>
#include <wiringPi.h>
#include <ros/ros.h>
#include <rcr2018/AngVel.h>

//サンプリングタイム
const double sampling_time = 0.01;

//const value
const double pi = 3.141592653589793;
const double gear_ratio = 2.73913043;

//エンコーダ（A相）が接続されているピン番号
const int encoder_pin = 14;

//経過時間を単位：秒で返す
double elapsed_sec(struct timespec& t_old, struct timespec& t_new)
{
    return ((double)(t_new.tv_sec - t_old.tv_sec) + ((double)(t_new.tv_nsec - t_old.tv_nsec) / (1000.0 * 1000.0 * 1000.0)));
}

int main(int argc, char **argv)
{
    //wiringPiの初期化
    if(wiringPiSetupGpio() == -1)
    {
      std::cout << "ERROR:Can't setup GPIO." << std::endl;
      return -1;
    }

    //エンコーダが接続されているピンを入力に設定
    pinMode(encoder_pin, INPUT);

    //ノード名の初期化
    ros::init(argc, argv, "rotaryencoder");

    //ROSシステムとの通信のためのノードハンドルを宣言
    ros::NodeHandle nh;

    //時間計測用構造体
    struct timespec t_start, t_now;

    /****************
    パブリッシャの宣言
    トピック名：ang_vel
    データ型：AngVel
    キューサイズ：1
    *****************/
    ros::Publisher ang_vel_pub = nh.advertise<rcr2018::AngVel>("ang_vel", 1);

    //パルスカウント数をカウントする変数
    unsigned long count = 0;

    //パルスがHighになっているときに連続カウントしないためのフラグ
    bool pulse_detect_flag = false;

    //ピンの入力格納用変数
    int pin_value;

    //計算した回転角速度を格納する変数
    //now:k時点の角速度 prev:(k-1)時点の角速度
    double ang_vel_now = 0.0;
    double ang_vel_prev = 0.0;

    //パブリッシュするデータ型の宣言
    //メンバ：double型 ang_vel
    rcr2018::AngVel msg;

    //経過時間を格納する変数
    double elapsed_time = 0.0;

    //Low pass filterの係数
    const double alpha = 0.5;

    //開始時刻を取得
    clock_gettime(CLOCK_MONOTONIC, &t_start);

    while(ros::ok())
    {
        //現時刻を取得
        clock_gettime(CLOCK_MONOTONIC, &t_now);

        //サンプリング周期以上経過していれば角速度を計算してパブリッシュ
        elapsed_time = elapsed_sec(t_start, t_now);

        if(elapsed_time >= sampling_time)
        {
             //角速度を計算
             ang_vel_now = ((count * 360.0 * pi) / (500.0 * 180.0 * gear_ratio)) / elapsed_time;

             //簡単なデジタルフィルタ(LPF)で角速度を平滑化しメッセージに格納
             msg.ang_vel = (alpha * ang_vel_now) + ((1 - alpha) * ang_vel_prev);

             //角速度を更新し、パルスカウント数をリセット
             ang_vel_prev = ang_vel_now;
             count = 0;

             //メッセージをパブリッシュ
             ang_vel_pub.publish(msg);

             //開始時刻を更新
             clock_gettime(CLOCK_MONOTONIC, &t_start);
        }

        //パルスカウント
        pin_value = digitalRead(encoder_pin);
        if(pin_value == 1 && !pulse_detect_flag)
        {
          pulse_detect_flag = true;
          count++;
        }

        //パルスが立ち下がったらフラグ解除
        if(pin_value == 0 && pulse_detect_flag)
        {
          pulse_detect_flag = false;
        }

    }

    return 0;
}
