// 阴影贴图回归测试。
// 编译运行（不在 CMake 里，手动跑）：
//   D:/mingw64/bin/g++.exe -std=c++17 -Isrc tests/test_shadow.cpp -o build/test_shadow.exe
//   ./build/test_shadow.exe
#include "soft_renderer.h"
#include <cstdio>
#include <cmath>

static double avg_brightness(const FrameBuffer& fb) {
    double s = 0;
    for (int y = 0; y < fb.height(); ++y)
        for (int x = 0; x < fb.width(); ++x) {
            Vector3 c = fb.pixel(x, y);
            s += (c.x + c.y + c.z) / 3.0;
        }
    return s / (fb.width() * fb.height());
}

int main() {
    int errors = 0;
    auto check = [&](bool ok, const char* what) {
        if (!ok) { std::printf("FAIL: %s\n", what); ++errors; }
    };

    ClipWindow win(1.0f);   // 全屏窗口：阴影区域不被窗口裁掉
    SolidMesh mesh;
    PointLight light;
    light.position = {3, 1.5f, -2};    // 右后侧光：悬浮球在地面左后方投出成片阴影

    FrameBuffer fb(300, 300);

    // ① 阴影开：逐像素亮度快照
    Camera cam;
    mesh.draw(fb, win, light, cam);
    std::vector<float> on(fb.width() * fb.height());
    for (int y = 0; y < fb.height(); ++y)
        for (int x = 0; x < fb.width(); ++x) {
            Vector3 c = fb.pixel(x, y);
            on[y * fb.width() + x] = (c.x + c.y + c.z) / 3.0f;
        }
    const double avg_on = avg_brightness(fb);

    // ② 阴影关：同样快照
    mesh.enable_shadows(false);
    mesh.draw(fb, win, light, cam);
    std::vector<float> off(fb.width() * fb.height());
    for (int y = 0; y < fb.height(); ++y)
        for (int x = 0; x < fb.width(); ++x) {
            Vector3 c = fb.pixel(x, y);
            off[y * fb.width() + x] = (c.x + c.y + c.z) / 3.0f;
        }
    const double avg_off = avg_brightness(fb);
    mesh.enable_shadows(true);

    // 断言一：阴影只会变暗——任何像素开阴影后都不得变亮
    int viol = 0;
    for (size_t i = 0; i < on.size(); ++i)
        if (on[i] > off[i] + 1e-4f) ++viol;
    check(viol == 0, "开阴影后任何像素都不得变亮");

    // 断言二：阴影真实存在——平均亮度下降，且显著变暗的像素成片出现
    int darker = 0;
    for (size_t i = 0; i < on.size(); ++i)
        if (off[i] - on[i] > 0.02f) ++darker;
    std::printf("avg_on=%.5f avg_off=%.5f diff=%.5f darker=%d\n", avg_on, avg_off, avg_off - avg_on, darker);
    check(avg_on < avg_off, "开阴影后平均亮度应下降");
    check(darker > 50, "应存在成片的阴影像素");

    // ③ 光源深度 pass：面向光源的悬浮球应写进深度图（地面按设计不投射
    //    阴影、不进光源深度图——它下方没有别的物体，见 render_depth_impl 注释）
    FrameBuffer sm(64, 64);
    PointLight side;  side.position = {5, 3, 0};
    Mat4 lview = Mat4::look_at(side.position, Vector3(0, 0, 0), Vector3(0, 1, 0));
    Mat4 lproj = Mat4::perspective(100.0f, 1.0f, 0.1f, 60.0f);
    mesh.render_depth(sm, lproj * lview);
    check(sm.depth_at(32, 32) < 1.0f, "光源深度图中心应有悬浮球");
    int filled = 0;
    for (int y = 0; y < 64; ++y)
        for (int x = 0; x < 64; ++x)
            if (sm.depth_at(x, y) < 1.0f) ++filled;
    std::printf("shadow map filled=%d/4096\n", filled);
    check(filled > 40, "深度图应有成片覆盖（悬浮球投影）");

    if (errors == 0) std::printf("shadow mapping regression test: PASS\n");
    return errors == 0 ? 0 : 1;
}
