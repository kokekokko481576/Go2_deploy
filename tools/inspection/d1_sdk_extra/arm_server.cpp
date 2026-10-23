// D1 アームの常駐プロセス。指令の送信と関節角の読み取りを1つのプロセスで続ける（2026-10-07）。
//
// set_joints / get_arm_joint_angle を毎回起動すると、1段ごとに
//   - set_joints: DDS のディスカバリー待ち 1秒 + 送信後 0.2秒
//   - get_arm_joint_angle: 4秒走らせて最後の行を読む
// がかかり、アームはとっくに着いているのに1段5秒になっていた（10/1 実機ログ: 伏せの展開9段で68秒）。
// さらに起動直後の Write() はディスカバリー前で届かないことがあり、再送で1回16秒失っていた。
// 常駐させれば publisher は一度だけマッチすればよく、角度も届いたそばから読める。
//
// 使い方（inspect_run.py が起動する。手で試すときも同じ）:
//   ./run.sh arm_server
//   標準入力に1行ずつ:
//     set <angle0> ... <angle6>   funcode 2 で送る（角度は度。get の servo0〜6 と同じ並び）
//     quit                        終了
//   標準出力:
//     ready                       起動してディスカバリーを待ち終えた
//     A <t> <a0> ... <a6>         関節角を受け取るたび（t は起動からの秒）
//     F <文字列>                  arm_Feedback を受け取ったとき
//     S <seq> <JSON>              送った
//     E <理由>                    入力が読めなかった
//
// **安全上の注意は set_joints.cpp と同じ。** このプロセスは来た角度をそのまま送る。
// 1関節ずつにする・干渉しない経路を選ぶのは呼び出し側（inspect_run.py / inspect_poses.json）の責任。
#include <chrono>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"

using namespace unitree::robot;

static std::mutex g_out;
static const auto g_t0 = std::chrono::steady_clock::now();

static double elapsed()
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - g_t0).count();
}

static void on_angles(const void *msg)
{
    const auto *m = static_cast<const unitree_arm::msg::dds_::PubServoInfo_ *>(msg);
    std::lock_guard<std::mutex> lk(g_out);
    std::printf("A %.3f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n", elapsed(),
                m->servo0_data_(), m->servo1_data_(), m->servo2_data_(), m->servo3_data_(),
                m->servo4_data_(), m->servo5_data_(), m->servo6_data_());
    std::fflush(stdout);
}

static void on_feedback(const void *msg)
{
    const auto *m = static_cast<const unitree_arm::msg::dds_::ArmString_ *>(msg);
    std::lock_guard<std::mutex> lk(g_out);
    std::printf("F %s\n", m->data_().c_str());
    std::fflush(stdout);
}

int main()
{
    ChannelFactory::Instance()->Init(0);
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher("rt/arm_Command");
    publisher.InitChannel();
    ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_> angles("current_servo_angle");
    angles.InitChannel(on_angles);
    ChannelSubscriber<unitree_arm::msg::dds_::ArmString_> feedback("arm_Feedback");
    feedback.InitChannel(on_feedback);

    // 最初の送信がディスカバリー前に落ちないよう、一度だけ待つ（set_joints.cpp と同じ理由）
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    {
        std::lock_guard<std::mutex> lk(g_out);
        std::printf("ready\n");
        std::fflush(stdout);
    }

    int seq = 100;
    std::string line;
    while (std::getline(std::cin, line)) {
        std::istringstream in(line);
        std::string cmd;
        in >> cmd;
        if (cmd == "quit") {
            break;
        }
        if (cmd != "set") {
            std::lock_guard<std::mutex> lk(g_out);
            std::printf("E 未対応のコマンド: %s\n", line.c_str());
            std::fflush(stdout);
            continue;
        }
        double a[7];
        bool ok = true;
        for (double &v : a) {
            if (!(in >> v)) {
                ok = false;
            }
        }
        if (!ok) {
            std::lock_guard<std::mutex> lk(g_out);
            std::printf("E 角度は7個: %s\n", line.c_str());
            std::fflush(stdout);
            continue;
        }
        // seq は毎回変える（同じ id の繰り返しを機体が重複とみなす例が Go2 本体にある）
        char buf[512];
        std::snprintf(buf, sizeof(buf),
                      "{\"seq\":%d,\"address\":1,\"funcode\":2,\"data\":{\"mode\":1,"
                      "\"angle0\":%.2f,\"angle1\":%.2f,\"angle2\":%.2f,\"angle3\":%.2f,"
                      "\"angle4\":%.2f,\"angle5\":%.2f,\"angle6\":%.2f}}",
                      ++seq, a[0], a[1], a[2], a[3], a[4], a[5], a[6]);
        unitree_arm::msg::dds_::ArmString_ msg{};
        msg.data_() = buf;
        publisher.Write(msg);
        std::lock_guard<std::mutex> lk(g_out);
        std::printf("S %d %s\n", seq, buf);
        std::fflush(stdout);
    }
    return 0;
}
