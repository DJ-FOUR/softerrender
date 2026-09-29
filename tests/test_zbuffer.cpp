// z-buffer 消隐的回归测试：深度测试必须与绘制顺序无关。
//
// 编译运行（不在 CMake 里，手动跑）：
//   D:/mingw64/bin/g++.exe -std=c++17 -Isrc tests/test_zbuffer.cpp -o build/test_zbuffer.exe
//   ./build/test_zbuffer.exe
#include "soft_renderer.h"
#include <cstdio>

static bool is_red(const Vector3& c)  { return c.x > 0.9f && c.y < 0.1f && c.z < 0.1f; }
static bool is_blue(const Vector3& c) { return c.x < 0.1f && c.y < 0.1f && c.z > 0.9f; }
static bool is_white(const Vector3& c){ return c.x > 0.99f && c.y > 0.99f && c.z > 0.99f; }

int main() {
    FrameBuffer fb(100, 100);
    const Vector3 red(1, 0, 0), blue(0, 0, 1);
    int errors = 0;
    auto check = [&](bool ok, const char* what) {
        if (!ok) { std::printf("FAIL: %s\n", what); ++errors; }
    };

    // 远矩形 z=0.8，近矩形 z=0.3，右下角互相重叠
    const std::vector<Vector3> far_rect  = {{20, 20, 0.8f}, {80, 20, 0.8f},
                                            {80, 80, 0.8f}, {20, 80, 0.8f}};
    const std::vector<Vector3> near_rect = {{40, 40, 0.3f}, {90, 40, 0.3f},
                                            {90, 90, 0.3f}, {40, 90, 0.3f}};

    // ① 先画远、后画近：重叠区应是近的蓝
    fb.fill_polygon(far_rect, red, 0);
    fb.fill_polygon(near_rect, blue, 1);
    check(is_red(fb.pixel(30, 30)),  "远矩形独占区应为红");
    check(is_blue(fb.pixel(60, 60)), "重叠区应为近的蓝（后画的近片元胜出）");
    check(is_blue(fb.pixel(85, 85)), "近矩形独占区应为蓝");

    // ② 先画近、后画远：z-buffer 的核心性质——结果与绘制顺序无关
    fb.clear();
    fb.fill_polygon(near_rect, blue, 1);
    fb.fill_polygon(far_rect, red, 0);
    check(is_blue(fb.pixel(60, 60)), "重叠区仍应为蓝（远片元深度测试失败被丢弃）");

    // ③ 关闭深度测试：退化为画家算法，后画的覆盖先画的
    fb.enable_depth_test(false);
    fb.clear();
    fb.fill_polygon(near_rect, blue, 1);
    fb.fill_polygon(far_rect, red, 0);
    check(is_red(fb.pixel(60, 60)), "深度关闭时后画的红应覆盖蓝（画家算法行为）");
    fb.enable_depth_test(true);

    // ④ 描边 id 门槛回归：藏在近面后面的另一个面，其描边不得越过近面领地。
    // 旧实现给描边深度加 0.002 偏置——在近共面的共享边上，真实深度差
    // (~0.0015) 小于偏置，后画面描边翻盘，漏线画在先画面的填充上。
    // id 方案下描边只画在"自己面的填充赢下"的像素上，歧义从根上消失。
    fb.clear();
    const std::vector<Vector3> cover_rect = {{20, 20, 0.3f}, {80, 20, 0.3f},
                                             {80, 80, 0.3f}, {20, 80, 0.3f}};
    const std::vector<Vector3> behind_rect = {{15, 15, 0.8f}, {85, 15, 0.8f},
                                              {85, 85, 0.8f}, {15, 85, 0.8f}};
    fb.fill_polygon(cover_rect, blue, 0);                     // 近面赢下 x∈[20,80)
    fb.fill_polygon(behind_rect, red, 1);                     // 远面只赢外圈 L 形
    fb.draw_line(20.1f, 50, 79.9f, 50, Vector3(0.0f, 0.5f, 0.0f), 1);  // 远面的描边横穿近面
    check(is_blue(fb.pixel(50, 50)),  "远面描边横穿处应保持近面蓝色");
    check(is_blue(fb.pixel(79, 50)),  "紧贴边界的像素不应出现远面描边");
    // 远面描边在自己赢下的像素上必须可见：远面左边缘外圈（近面外）
    check(is_red(fb.pixel(17, 50)),   "远面独占区应为红（填充）");
    fb.draw_line(15.1f, 12, 84.9f, 12, Vector3(0.0f, 0.5f, 0.0f), 1);  // 远面顶边（无主区）
    check(fb.pixel(50, 12).y > 0.4f && fb.pixel(50, 12).x < 0.1f,
          "远面在无主区的描边应可见");

    if (errors == 0) std::printf("z-buffer regression test: PASS\n");
    return errors == 0 ? 0 : 1;
}
