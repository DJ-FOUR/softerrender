// 无头帧导出调试工具：不走窗口，直接把 SolidMesh 渲染的两帧存成 24 位 BMP。
// 用途：快速目检模型导入 / 光照 / 阴影效果，也可用图片查看器对比改动前后。
// 编译运行（不在 CMake 里，手动跑）：
//   D:/mingw64/bin/g++.exe -std=c++17 -Isrc tests/dump_frame.cpp -o build/dump_frame.exe
//   ./build/dump_frame.exe            # 在 build/ 下生成 frame_*.bmp
#include "soft_renderer.h"
#include <cstdint>
#include <cstdio>

static void write_bmp(const char* path, const FrameBuffer& fb) {
    const int w = fb.width(), h = fb.height();
    const int row = w * 3;                       // 800×3=2400，天然 4 字节对齐
    const uint32_t data_size = uint32_t(row * h);
    const uint32_t file_size = 14 + 40 + data_size;

    uint8_t hdr[54] = {0};
    hdr[0] = 'B'; hdr[1] = 'M';
    *reinterpret_cast<uint32_t*>(&hdr[2])  = file_size;
    *reinterpret_cast<uint32_t*>(&hdr[10]) = 54;
    *reinterpret_cast<uint32_t*>(&hdr[14]) = 40;
    *reinterpret_cast<int32_t*>(&hdr[18])  = w;
    *reinterpret_cast<int32_t*>(&hdr[22])  = h;
    *reinterpret_cast<uint16_t*>(&hdr[26]) = 1;
    *reinterpret_cast<uint16_t*>(&hdr[28]) = 24;
    *reinterpret_cast<uint32_t*>(&hdr[34]) = data_size;

    FILE* f = std::fopen(path, "wb");
    if (!f) { std::printf("cannot write %s\n", path); return; }
    std::fwrite(hdr, 1, 54, f);
    std::vector<uint8_t> rowbuf(static_cast<size_t>(row));
    for (int y = h - 1; y >= 0; --y) {           // BMP 自下而上
        for (int x = 0; x < w; ++x) {
            const Vector3 c = fb.pixel(x, y);
            rowbuf[size_t(x * 3 + 0)] = uint8_t(std::min(1.0f, c.z) * 255.0f + 0.5f); // B
            rowbuf[size_t(x * 3 + 1)] = uint8_t(std::min(1.0f, c.y) * 255.0f + 0.5f); // G
            rowbuf[size_t(x * 3 + 2)] = uint8_t(std::min(1.0f, c.x) * 255.0f + 0.5f); // R
        }
        std::fwrite(rowbuf.data(), 1, size_t(row), f);
    }
    std::fclose(f);
    std::printf("written %s\n", path);
}

// 由 eye/target 推导 FPS 相机的 yaw/pitch（与 test_shadow_stability 相同的辅助）
static Camera make_camera(const Vector3& eye, const Vector3& target) {
    Camera c;
    c.position = eye;
    const Vector3 d = (target - eye).normalized();
    c.yaw   = std::atan2(d.x, -d.z) * 57.29578f;
    c.pitch = std::asin(d.y) * 57.29578f;
    return c;
}

int main() {
    FrameBuffer fb(800, 800);
    ClipWindow win(1.0f);
    SolidMesh mesh;
    PointLight light;

    mesh.draw(fb, win, light, Camera{});         // 默认机位（正对场景）
    write_bmp("frame_default.bmp", fb);

    mesh.draw(fb, win, light, make_camera({6, 4, 8}, {0, 0.5f, 0}));   // 侧后机位
    write_bmp("frame_side.bmp", fb);
    return 0;
}
