// 7関節の角度を引数で指定して送る（funcode 2 = 複数関節の同時指令）。
//
// 既存サンプルは角度がハードコードで、符号や可動域の照合に使えないため作った（2026-09-23）。
//
// 使い方:
//   ./run.sh set_joints <angle0> <angle1> <angle2> <angle3> <angle4> <angle5> <angle6>
//   角度は度。`get_arm_joint_angle` が出す servo0〜servo6 と同じ並び・同じ符号。
//
// **安全上の注意**
//   - 現在角を `get_arm_joint_angle` で読み、**動かしたい1軸だけを変えて残りは現在値を渡す**。
//     全軸に離れた値を入れると全部が同時に動く。
//   - 1回の変化は小さく（10〜20度）。アームはGo2の背面に載っており、
//     大きく振ると機体や床に当たる。
//   - simでは多関節同時指令でj1が可動域上限に張り付く事象がある（4軸同時で3回とも再現、
//     原因未特定）。**実機では1軸ずつ変えること。**
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>

#include "msg/ArmString_.hpp"

#define TOPIC "rt/arm_Command"
using namespace unitree::robot;

int main(int argc, char **argv)
{
    if (argc != 8) {
        std::fprintf(stderr,
                     "使い方: %s <angle0> <angle1> <angle2> <angle3> <angle4> <angle5> <angle6>\n"
                     "  角度は度。get_arm_joint_angle の servo0〜servo6 と同じ並び。\n"
                     "  動かしたい軸だけ変え、残りは現在値を渡すこと。\n", argv[0]);
        return 1;
    }
    double a[7];
    for (int i = 0; i < 7; ++i) {
        a[i] = std::atof(argv[i + 1]);
    }

    ChannelFactory::Instance()->Init(0);
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC);
    publisher.InitChannel();
    // DDSのディスカバリーが済む前に Write() すると届かないことがある
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    char buf[512];
    std::snprintf(buf, sizeof(buf),
                  "{\"seq\":4,\"address\":1,\"funcode\":2,\"data\":{\"mode\":1,"
                  "\"angle0\":%.2f,\"angle1\":%.2f,\"angle2\":%.2f,\"angle3\":%.2f,"
                  "\"angle4\":%.2f,\"angle5\":%.2f,\"angle6\":%.2f}}",
                  a[0], a[1], a[2], a[3], a[4], a[5], a[6]);
    std::printf("送信: %s\n", buf);

    unitree_arm::msg::dds_::ArmString_ msg{};
    msg.data_() = buf;
    publisher.Write(msg);
    // Write() 直後に終了すると送信が完了しないことがある
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    return 0;
}
