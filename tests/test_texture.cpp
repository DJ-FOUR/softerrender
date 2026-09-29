// 纹理映射回归测试：① 双线性采样数学 ② UV 透视校正插值 ③ 真实资产读图。
// 编译运行（不在 CMake 里，手动跑）：
//   D:/mingw64/bin/g++.exe -std=c++17 -Isrc tests/test_texture.cpp -o build/test_texture.exe
//   ./build/test_texture.exe
#include "soft_renderer.h"
#include <cstdio>
#include <cmath>

int main() {
    int errors = 0;
    auto check = [&](bool ok, const char* what) {
        if (!ok) { std::printf("FAIL: %s\n", what); ++errors; }
    };
    auto near1 = [](float a, float b, float eps) { return std::fabs(a - b) < eps; };

    // ========== ① 双线性采样：2×2 纹理，四个纯色角 ==========
    {
        Texture t;
        t.width = t.height = 2;
        t.rgb = {
            255, 0,   0,    // (0,0) 左上：红
            0,   255, 0,    // (1,0) 右上：绿
            0,   0,   255,  // (0,1) 左下：蓝
            255, 255, 0,    // (1,1) 右下：黄
        };
        // v 轴翻转检查：v=0 应取图像最后一行（蓝/黄那一行）
        const Vector3 bottom = sample_texture(t, 0.5f, 0.0f);
        check(near1(bottom.x, 0.5f, 1e-4f) && near1(bottom.z, 0.5f, 1e-4f),
              "v=0 应采样图像底部行（蓝黄混合）");
        const Vector3 top = sample_texture(t, 0.5f, 1.0f);
        check(near1(top.x, 0.5f, 1e-4f) && near1(top.y, 0.5f, 1e-4f),
              "v=1 应采样图像顶部行（红绿混合）");
        // 双线性：纹理正中心 = 四角平均
        const Vector3 mid = sample_texture(t, 0.5f, 0.5f);
        check(near1(mid.x, 0.5f, 1e-4f) && near1(mid.y, 0.5f, 1e-4f) && near1(mid.z, 0.25f, 1e-4f),
              "纹理中心应为四角双线性平均");
        // 回绕：u=1.25 等价 u=0.25
        const Vector3 wrap = sample_texture(t, 1.25f, 0.0f);
        const Vector3 ref = sample_texture(t, 0.25f, 0.0f);
        check(near1(wrap.x, ref.x, 1e-4f) && near1(wrap.y, ref.y, 1e-4f) && near1(wrap.z, ref.z, 1e-4f),
              "u 超出 [0,1] 应回绕");
    }

    // ========== ② UV 透视校正插值（PolyVert 的 (u/w,1/w) 通道）==========
    {
        FrameBuffer fb(101, 104);
        // 一条水平细带，左端 w=1（inv_w=1），右端 w=0.5（inv_w=2）。
        // u 取 0→1：顶点存 u/w → 左 0、右 2。屏幕线性插值到中点：
        //   uv.x = 1, inv_w = 1.5 → u = 1/1.5 ≈ 0.667（透视校正）
        //   若错误地直接线性插 u（0.5）即"贴图游动"的老 bug。
        std::vector<PolyVert> strip = {
            {{0,   50, 0.5f}, {0, 0, 0}, {0.0f, 0.0f}, 1.0f},
            {{100, 50, 0.5f}, {0, 0, 0}, {2.0f, 0.0f}, 2.0f},
            {{100, 52, 0.5f}, {0, 0, 0}, {2.0f, 0.0f}, 2.0f},
            {{0,   52, 0.5f}, {0, 0, 0}, {0.0f, 0.0f}, 1.0f},
        };
        fb.fill_polygon(strip, 3, [](float, float, float, const Vector3&,
                                     float u, float v) {
            return Vector3(u, v, 0);
        });
        const Vector3 c = fb.pixel(50, 50);
        check(near1(c.x, 0.6667f, 0.03f),
              "中点 u 应为透视校正的 0.667（而非线性插值的 0.5）");
        check(near1(c.y, 0.0f, 1e-4f), "v 恒为 0");
        const Vector3 r = fb.pixel(98, 50);   // 靠近右端 → u→1
        check(r.x > 0.9f, "右端附近 u 应接近 1");
    }

    // ========== ③ 真实资产：src/tietu.png 能读入 ==========
    {
        Texture t;
        for (const char* p : {"src/tietu.png", "../src/tietu.png"})
            if ((t = load_texture(p)).ok()) break;
        check(t.ok(), "src/tietu.png 应能通过 stb_image 读入");
        if (t.ok())
            std::printf("tietu.png: %dx%d, %zu bytes\n", t.width, t.height, t.rgb.size());
    }

    if (errors == 0) std::printf("texture mapping regression test: PASS\n");
    return errors == 0 ? 0 : 1;
}
