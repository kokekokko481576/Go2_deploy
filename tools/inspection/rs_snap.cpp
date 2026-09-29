// RealSense を1枚撮って保存する（2026-09-24。Go2 背中の Jetson で D435i を確かめるために作成）。
//
// Jetson はインターネットに出られず pyrealsense2 を入れられないので、ROS noetic に
// 同梱の librealsense2 (2.50) に直接リンクする。OpenCV にも依存しない。
//
//   g++ -O2 -std=c++14 rs_snap.cpp -o rs_snap -I/opt/ros/noetic/include \
//       -L/opt/ros/noetic/lib/aarch64-linux-gnu -lrealsense2 \
//       -Wl,-rpath,/opt/ros/noetic/lib/aarch64-linux-gnu
//   ./rs_snap <出力の接頭辞> [深度の幅 深度の高さ]
//     → <接頭辞>_color.ppm / _depth.pgm(16bit, mm) / _info.txt
//
// 深度はカラーに位置合わせ(align)してから保存するので、画素がカラーと1対1で対応する。
// **深度の撮影解像度を変えても、保存される深度はカラーと同じ 1280x720 になる。**
//
// 深度の解像度（2026-09-28 追加。省略時は従来どおり 1280x720）:
//   1280x720 では最短測距が約0.28m。アップ撮影（約0.2m）では深度の9割以上が抜けた。
//   D435 系の最短測距は深度の解像度に比例して縮む（視差の探索幅が画素数で決まるため）ので、
//   848x480 にすると最短測距が縮む。**どこまで縮むかは実機で確かめること。**
//   例: ./rs_snap snap_848 848 480
#include <librealsense2/rs.hpp>

#include <algorithm>
#include <cstdlib>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

static void write_ppm(const std::string &path, const rs2::video_frame &f) {
    std::ofstream o(path, std::ios::binary);
    o << "P6\n" << f.get_width() << " " << f.get_height() << "\n255\n";
    o.write(static_cast<const char *>(f.get_data()), f.get_width() * f.get_height() * 3);
}

// 16bit PGM はビッグエンディアン。値は mm（depth_scale を掛けて換算済み）
static void write_pgm16(const std::string &path, const std::vector<uint16_t> &mm, int w, int h) {
    std::ofstream o(path, std::ios::binary);
    o << "P5\n" << w << " " << h << "\n65535\n";
    for (uint16_t v : mm) {
        char b[2] = {static_cast<char>(v >> 8), static_cast<char>(v & 0xff)};
        o.write(b, 2);
    }
}

int main(int argc, char **argv) try {
    const std::string prefix = argc > 1 ? argv[1] : "snap";
    const int W = 1280, H = 720;             // カラー
    int DW = W, DH = H;                      // 深度
    if (argc == 4) {
        DW = std::atoi(argv[2]);
        DH = std::atoi(argv[3]);
    } else if (argc != 1 && argc != 2) {
        std::fprintf(stderr, "使い方: %s <出力の接頭辞> [深度の幅 深度の高さ]\n", argv[0]);
        return 2;
    }

    rs2::context ctx;
    auto devs = ctx.query_devices();
    if (devs.size() == 0) {
        std::fprintf(stderr, "RealSense が見つからない\n");
        return 1;
    }
    auto dev = devs[0];
    std::printf("機種: %s  シリアル: %s  FW: %s  USB: %s\n",
                dev.get_info(RS2_CAMERA_INFO_NAME), dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER),
                dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION),
                dev.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)
                    ? dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) : "?");

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, W, H, RS2_FORMAT_RGB8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, DW, DH, RS2_FORMAT_Z16, 30);
    rs2::pipeline pipe;
    auto prof = pipe.start(cfg);
    const float scale = prof.get_device().first<rs2::depth_sensor>().get_depth_scale();

    // 自動露出が落ち着くまで捨てる
    for (int i = 0; i < 30; ++i) pipe.wait_for_frames();

    rs2::align to_color(RS2_STREAM_COLOR);
    auto fs = to_color.process(pipe.wait_for_frames());
    auto color = fs.get_color_frame();
    auto depth = fs.get_depth_frame();
    pipe.stop();

    const int w = depth.get_width(), h = depth.get_height();
    const auto *raw = static_cast<const uint16_t *>(depth.get_data());
    std::vector<uint16_t> mm(w * h);
    std::vector<float> valid;
    for (int i = 0; i < w * h; ++i) {
        const float m = raw[i] * scale;
        mm[i] = static_cast<uint16_t>(std::min(65535.0f, m * 1000.0f + 0.5f));
        if (raw[i]) valid.push_back(m);
    }
    // 中央 21x21 画素の中央値
    std::vector<float> center;
    for (int y = h / 2 - 10; y <= h / 2 + 10; ++y)
        for (int x = w / 2 - 10; x <= w / 2 + 10; ++x)
            if (raw[y * w + x]) center.push_back(raw[y * w + x] * scale);
    auto median = [](std::vector<float> v) {
        if (v.empty()) return 0.0f;
        std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
        return v[v.size() / 2];
    };

    write_ppm(prefix + "_color.ppm", color);
    write_pgm16(prefix + "_depth.pgm", mm, w, h);

    auto in = color.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    std::ofstream info(prefix + "_info.txt");
    char buf[512];
    std::snprintf(buf, sizeof buf,
                  "color %dx%d  fx=%.2f fy=%.2f cx=%.2f cy=%.2f\n"
                  "depth_stream %dx%d（保存はカラーに位置合わせ）  depth_scale=%.6f m/unit\n"
                  "有効な深度画素 %.1f%%  中央21x21の中央値 %.3f m  有効画素の中央値 %.3f m\n",
                  in.width, in.height, in.fx, in.fy, in.ppx, in.ppy, DW, DH, scale,
                  100.0 * valid.size() / (w * h), median(center), median(valid));
    info << buf;
    std::fputs(buf, stdout);
    return 0;
} catch (const rs2::error &e) {
    std::fprintf(stderr, "RealSense エラー: %s (%s)\n", e.what(), e.get_failed_function().c_str());
    return 1;
}
