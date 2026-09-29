// Blinn-Phong 光照回归测试。
// 编译运行（不在 CMake 里，手动跑）：
//   D:/mingw64/bin/g++.exe -std=c++17 -Isrc tests/test_light.cpp -o build/test_light.exe
//   ./build/test_light.exe
#include "soft_renderer.h"
#include <cstdio>
#include <cmath>

int main() {
    int errors = 0;
    auto check = [&](bool ok, const char* what) {
        if (!ok) { std::printf("FAIL: %s\n", what); ++errors; }
    };

    const Vector3 albedo(0.8f, 0.4f, 0.2f);
    const Vector3 pos(0, 0, 0);
    const Vector3 n_up(0, 1, 0);

    // ① 正对光源：漫反射满额，明显比纯环境光亮
    PointLight light;  light.position = {0, 5, 0};
    const Vector3 eye(0, 0, 10);
    Vector3 lit = blinn_phong(pos, n_up, albedo, light, eye);
    Vector3 amb_only = albedo * light.ambient;
    check(lit.x > amb_only.x + 0.1f, "正对光源应比纯环境光亮");

    // ② 背对光源：N·L ≤ 0，漫反射和镜面都为 0，只剩环境光
    Vector3 dark = blinn_phong(pos, Vector3(0, -1, 0), albedo, light, eye);
    check(std::fabs(dark.x - amb_only.x) < 1e-4f, "背对光源应只剩环境光");

    // ③ 镜面高光：视线与光线都沿法线方向 → H 与 N 重合 → (N·H)^s = 1 满额
    PointLight spec_light;  spec_light.position = {0, 5, 0};
    Vector3 spec = blinn_phong(pos, n_up, albedo, spec_light, Vector3(0, 10, 0));
    check(spec.x > lit.x, "H 与 N 重合时高光应比只有漫反射亮");
    check(spec.x <= 1.0f && spec.z <= 1.0f, "结果应钳制在 [0,1]");

    // ④ 移动光源（位置是唯一状态）：同一面着色随之改变
    PointLight moved;  moved.position = {-5, 2, 0};
    Vector3 c2 = blinn_phong(pos, n_up, albedo, moved, eye);
    check(std::fabs(lit.x - c2.x) > 1e-3f, "移动光源应改变着色结果");
    // 移到正下方：N·L = -1 < 0，与背对光源等价
    PointLight below;  below.position = {0, -5, 0};
    Vector3 c3 = blinn_phong(pos, n_up, albedo, below, eye);
    check(std::fabs(c3.x - amb_only.x) < 1e-4f, "光源移到面正下方应只剩环境光");

    // ⑤ 光源小球：光源坐标处应渲染出亮黄色自发光小球
    // （它不参与阴影贴图 pass——否则小球会包住光源，全场皆影）
    PointLight default_light;                 // 默认位置 (1.2, 2.0, 1.8)
    FrameBuffer fb(300, 300);
    ClipWindow win(1.0f);
    SolidMesh mesh;
    Camera cam;
    mesh.draw(fb, win, default_light, cam);
    int glow = 0;
    for (int y = 0; y < 300; ++y)
        for (int x = 0; x < 300; ++x) {
            Vector3 c = fb.pixel(x, y);
            if (c.x > 0.95f && c.y > 0.75f && c.y < 0.95f && c.z > 0.15f && c.z < 0.35f)
                ++glow;
        }
    check(glow > 30, "光源坐标处应渲染出亮黄色小球");

    if (errors == 0) std::printf("blinn-phong regression test: PASS\n");
    return errors == 0 ? 0 : 1;
}
