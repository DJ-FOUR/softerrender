// 法线插值（平滑着色）回归测试：覆盖法线穿过的三段管线——
//   ① 视锥体裁剪（clip_polygon_frustum_n）：跨平面处法线随位置插值
//   ② Weiler–Atherton（带法线版）：交点法线沿 subject 边插值
//   ③ AET 填充（PolyVert 版 fill_polygon）：逐像素法线插值
// 编译运行（不在 CMake 里，手动跑）：
//   D:/mingw64/bin/g++.exe -std=c++17 -Isrc tests/test_normal.cpp -o build/test_normal.exe
//   ./build/test_normal.exe
#include "soft_renderer.h"
#include <cstdio>
#include <cmath>

int main() {
    int errors = 0;
    auto check = [&](bool ok, const char* what) {
        if (!ok) { std::printf("FAIL: %s\n", what); ++errors; }
    };
    auto near3 = [](const Vector3& a, const Vector3& b, float eps) {
        return std::fabs(a.x - b.x) < eps && std::fabs(a.y - b.y) < eps &&
               std::fabs(a.z - b.z) < eps;
    };

    // ========== ① 视锥体裁剪：三角形跨近平面，角点法线随交点插值 ==========
    {
        // 三个顶点：v0 在近平面内 (w=1)，v1 在外 (w=-1)，v2 在内 (w=2)
        std::vector<ClipVert> tri = {
            {{ 1,  1, 0, 1}, {0, 0, 1}},     // 顶点法线互不相同，插值可辨
            {{-1, -1, 0, -1}, {0, 1, 0}},
            {{ 1, -1, 0, 2}, {1, 0, 0}},
        };
        // w >= 0.5 半空间裁剪产生两个交点：
        //   v0-v1 边 t=(0.5-1)/(-1-1)=0.25 → 法线 (0,0,1)+0.25·((0,1,0)-(0,0,1)) = (0,0.25,0.75)
        //   v1-v2 边 t=(0.5+1)/(2+1)=0.5  → 法线 (0,1,0)+0.5·((1,0,0)-(0,1,0)) = (0.5,0.5,0)
        std::vector<ClipVert> out = clip_polygon_frustum_n(tri, 0.5f);
        check(out.size() == 4, "三角形跨单平面裁剪后应为四边形");
        int crossings = 0;
        bool n01_ok = false, n12_ok = false;
        for (const ClipVert& v : out)
            if (std::fabs(v.c.w - 0.5f) < 1e-6f) {
                ++crossings;
                if (near3(v.n, Vector3(0, 0.25f, 0.75f), 1e-4f)) n01_ok = true;
                if (near3(v.n, Vector3(0.5f, 0.5f, 0), 1e-4f)) n12_ok = true;
            }
        check(crossings == 2, "跨单平面裁剪应产生 2 个交点");
        check(n01_ok, "v0-v1 交点处法线应按 t=0.25 插值");
        check(n12_ok, "v1-v2 交点处法线应按 t=0.5 插值");
    }

    // ========== ② AET 填充：四边形上法线从 (1,0,0) 渐变到 (0,0,1) ==========
    {
        FrameBuffer fb(101, 101);
        // 屏幕四边形 (0,0)~(100,100)，z 恒 0.5；左下角法线 (1,0,0)、右上角 (0,0,1)，
        // 其余两角取 x+y 平均使法线场为线性函数 n ∝ (1-t, 0, t)（t = (x+y)/200）
        std::vector<PolyVert> quad = {
            {{0,   0,   0.5f}, {1,    0, 0}},
            {{100, 0,   0.5f}, {0.5f, 0, 0.5f}},
            {{100, 100, 0.5f}, {0,    0, 1}},
            {{0,   100, 0.5f}, {0.5f, 0, 0.5f}},
        };
        // shader 返回归一化法线的 z 分量；中心像素 (50,50) 处 t=0.5，
        // 归一化后 n=(0.5,0,0.5)/|·| → z = 0.5/√0.5 ≈ 0.7071
        fb.fill_polygon(quad, 7, [](float, float, float, const Vector3& nrm,
                                    float, float) {
            const Vector3 u = nrm.normalized();
            return Vector3(u.z, u.z, u.z);
        });
        const Vector3 c = fb.pixel(50, 50);
        check(std::fabs(c.x - 0.7071f) < 0.02f,
              "四边形中心像素的插值法线应为 (1,0,0)/(0,0,1) 的对称混合");
        const Vector3 l = fb.pixel(5, 5);      // 左下角附近 t≈0.05 → z≈0.05
        check(l.x < 0.09f, "左下角附近应仍以 (1,0,0) 法线为主");
        const Vector3 r = fb.pixel(95, 95);    // 右上角附近 t≈0.95 → z≈0.95
        check(r.x > 0.91f, "右上角附近应仍以 (0,0,1) 法线为主");
    }

    // ========== ③ W-A：矩形窗口裁剪一个带渐变法线的多边形 ==========
    {
        // 三角形横跨窗口左边界：v0→v1 边垂直穿过 x=-50 于 (-50, 0)，
        // t = 50/200 = 0.25 → 法线 = (1,0,0)+0.25·((0,1,0)-(1,0,0)) = (0.75,0.25,0)
        Rect win{-50, -50, 50, 50};
        std::vector<PolyVert> tri = {
            {{-100,  0, 0.5f}, {1, 0, 0}},
            {{ 100,  0, 0.5f}, {0, 1, 0}},
            {{   0, 90, 0.5f}, {0, 0, 1}},
        };
        auto loops = weiler_atherton(tri, win);
        check(loops.size() == 1, "凸三角形被窗口裁剪应输出一个环");
        bool found = false;
        for (const auto& loop : loops)
            for (const PolyVert& p : loop)
                if (std::fabs(p.pos.x + 50.0f) < 1e-3f && std::fabs(p.pos.y) < 1e-3f) {
                    found = true;
                    check(near3(p.nrm, Vector3(0.75f, 0.25f, 0), 1e-3f),
                          "W-A 交点处法线应沿 subject 边按 t=0.25 线性插值");
                }
        check(found, "输出环应包含窗口左边界上的交点 (-50, 0)");
    }

    if (errors == 0) std::printf("normal interpolation regression test: PASS\n");
    return errors == 0 ? 0 : 1;
}
