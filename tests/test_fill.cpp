// AET 扫描线填充的回归测试：近水平薄边的外推 bug。
//
// 复现：多边形顶边在 y 向厚度仅 0.05px（近水平薄边，斜率倒数 1/m = −3600）。
// bug 版本会把这条边放进"不含任何采样高度"的扫描线桶，交点 x 沿边方向
// 外推 360px，把填充 span 配对到错误一侧——屏幕上表现为横贯画面的长横线。
//
// 编译运行（不在 CMake 里，手动跑）：
//   D:/mingw64/bin/g++.exe -std=c++17 -Isrc tests/test_fill.cpp -o build/test_fill.exe
//   ./build/test_fill.exe
#include "soft_renderer.h"
#include <cstdio>

static bool is_white(const Vector3& c) {
    return c.x > 0.99f && c.y > 0.99f && c.z > 0.99f;
}
static bool is_red(const Vector3& c) {
    return c.x > 0.9f && c.y < 0.1f && c.z < 0.1f;
}

int main() {
    FrameBuffer fb(200, 200);

    // 四边形：顶边 A→B 的 y 向厚度仅 0.05px，dx = 180
    const std::vector<Vector3> poly = {
        {190, 100.40f, 0},   // A
        {10,  100.45f, 0},   // B
        {10,  130.00f, 0},   // C
        {190, 130.00f, 0},   // D
    };
    const Vector3 red(1, 0, 0);
    fb.fill_polygon(poly, red, 0);

    int errors = 0;
    auto check = [&](bool ok, const char* what) {
        if (!ok) { std::printf("FAIL: %s\n", what); ++errors; }
    };

    // 顶行 k=100（采样高度 100.5）：span 应为 [10, 190)，薄边不得产生假交点
    check(is_white(fb.pixel(5, 100)),   "row100 x=5 应为背景（bug 版被假 span 涂红）");
    check(is_red(fb.pixel(50, 100)),    "row100 x=50 应填充");
    check(is_red(fb.pixel(189, 100)),   "row100 x=189 应填充");
    check(is_red(fb.pixel(50, 115)),    "row115 x=50 应填充");
    check(is_white(fb.pixel(5, 115)),   "row115 x=5 应为背景");
    check(is_white(fb.pixel(195, 115)), "row115 x=195 应为背景");

    // 全图扫描：红色像素不允许越出多边形范围
    for (int y = 0; y < 200; ++y)
        for (int x = 0; x < 200; ++x)
            if (is_red(fb.pixel(x, y)))
                check(x >= 10 && x < 190 && y >= 100 && y <= 130, "红像素越界");

    if (errors == 0) std::printf("AET fill regression test: PASS\n");
    return errors == 0 ? 0 : 1;
}
