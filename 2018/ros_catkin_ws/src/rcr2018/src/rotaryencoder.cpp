#include "ros/ros.h"
#include "rcr2018/AngVel.h"

int main(int argc, char **argv)
{
    //ノード名の初期化
    ros::init(argc, argv, "rotaryencoder");

    //ROSシステムとの通信のためのノードハンドルを宣言
    ros::NodeHandle nh;

    /****************
    パブリッシャの宣言
    トピック名：ang_vel
    データ型：AngVel
    キューサイズ：1
    *****************/
    ros::Publisher ang_vel_pub =
      nh.advertise<rcr2018::AngVel>("ang_vel", 1);
    
    //ループ周期を10Hzに設定
    ros::Rate loop_late(10);

    //パルスカウント数をカウントする変数
    unsigned long count = 0;

    //パブリッシュするデータ型の宣言
    rcr2018::AngVel msg;

    //TODO:ロータリーエンコーダから角速度を取得してパブリッシュするようにする
    while(ros::ok())
    {
        loop_rate.sleep();
    }

    return 0;
}