#include <chrono>
#include <thread>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"

#define TOPIC "rt/arm_Command"

using namespace unitree::robot;
using namespace unitree::common;

int main()
{
    ChannelFactory::Instance()->Init(0);
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC);
    publisher.InitChannel();

    // Publisher/Subscriber間のDDSディスカバリーが完了する前にWrite()すると
    // メッセージが届かないことがあるため、少し待ってから送信する
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    unitree_arm::msg::dds_::ArmString_ msg{};
    msg.data_() = "{\"seq\":4,\"address\":1,\"funcode\":5,\"data\":{\"mode\":0}}";
    publisher.Write(msg);

    // Write()直後にプロセスが終了すると送信が完了しないことがあるため、
    // 少し待ってから終了する
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    return 0;
}