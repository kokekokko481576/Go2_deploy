// ROS2 の /arm_Command が、D1 SDK と同じ DDS トピック・型として見えるかを確かめる検査用プログラム。
//
// **アーム実機は要らない。** このプログラムは機体と同じ側に立って
// `rt/arm_Command` を購読するだけなので、同じPC上で d1_arm_bridge を動かして
// JSON が届けば「ROS2 から送ったコマンドは機体にも届く」ことが確かめられる。
//
// 使い方(driverコンテナ内、run.sh 経由で LD_LIBRARY_PATH を効かせる):
//   /root/d1_bridge_tools/run_probe.sh
// 別端末で:
//   ros2 run d1_arm_bridge arm_bridge_node --ros-args -r arm_command_out:=/arm_Command -p dry_run:=false
//   ros2 topic pub --once /arm_command std_msgs/msg/Float64MultiArray "{data: [0.1,0,0,0,0,0,0,0]}"
#include <iostream>
#include <unistd.h>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include "msg/ArmString_.hpp"

#define TOPIC "rt/arm_Command"

using namespace unitree::robot;

static int g_count = 0;

void Handler(const void* msg)
{
    const unitree_arm::msg::dds_::ArmString_* pm = (const unitree_arm::msg::dds_::ArmString_*)msg;
    std::cout << "[probe] 受信 #" << ++g_count << ": " << pm->data_() << std::endl;
}

int main(int argc, char** argv)
{
    int seconds = (argc > 1) ? atoi(argv[1]) : 30;
    ChannelFactory::Instance()->Init(0);
    ChannelSubscriber<unitree_arm::msg::dds_::ArmString_> subscriber(TOPIC);
    subscriber.InitChannel(Handler);
    std::cout << "[probe] " << TOPIC << " を購読中（"
              << seconds << "秒）。型: unitree_arm::msg::dds_::ArmString_" << std::endl;
    sleep(seconds);
    std::cout << "[probe] 受信件数: " << g_count << std::endl;
    return g_count > 0 ? 0 : 1;
}
