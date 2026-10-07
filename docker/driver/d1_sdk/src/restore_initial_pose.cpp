#include <chrono>
#include <thread>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"

#define TOPIC "rt/arm_Command"

using namespace unitree::robot;
using namespace unitree::common;

// 本セッションでアームを動かす前に取得した初期姿勢に復帰させるための
// ワンショットコマンド。
// (servo0:91.4, servo1:-89.3, servo2:91.2, servo3:-0.3, servo4:7.9, servo5:1.3, servo6:-8)
int main()
{
    ChannelFactory::Instance()->Init(0);
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC);
    publisher.InitChannel();

    // Publisher/Subscriber間のDDSディスカバリーが完了する前にWrite()すると
    // メッセージが届かないことがあるため、少し待ってから送信する
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    unitree_arm::msg::dds_::ArmString_ msg{};
    msg.data_() = "{\"seq\":4,\"address\":1,\"funcode\":2,\"data\":{\"mode\":1,\"angle0\":91.4,\"angle1\":-89.3,\"angle2\":91.2,\"angle3\":-0.3,\"angle4\":7.9,\"angle5\":1.3,\"angle6\":-8}}";
    publisher.Write(msg);

    // Write()直後にプロセスが終了すると送信が完了しないことがあるため、
    // 少し待ってから終了する
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    return 0;
}
