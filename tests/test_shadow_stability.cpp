// 阴影视点稳定性回归测试（几何无关版）。
// 守护一个实测 bug：逐像素着色时用"屏幕坐标 + NDC 深度"重建片元世界
// 坐标（供 Blinn-Phong 与阴影图采样使用），若视线基向量误用"相机指向
// 世界原点"的方向（而非 cam.forward()），默认机位恰好盯着原点、两方向
// 重合，看不出异常；一旦相机移动/旋转，重建的世界坐标整体错位——阴影
// 和光照便随视点游动。
//
// 方法：光源固定在场景正上方，由对称性影子必然落在模型正下方的地面上。
// 用两台"不瞄准原点"的相机各渲染一对开/关阴影的图像，差分得到影子像素
// 的加权重心；把两台相机各自的重心沿像素射线反投影到地面平面，两个世界
// 坐标必须重合——这正是"阴影不随视点游动"的定义。地面高度与模型形状
// 从 SolidMesh 实际装配的场景里读出（内置回退 / OBJ 导入都适用）。
//
// 编译运行（不在 CMake 里，手动跑）：
//   D:/mingw64/bin/g++.exe -std=c++17 -Isrc tests/test_shadow_stability.cpp -o build/test_shadow_stability.exe
//   ./build/test_shadow_stability.exe
#include "soft_renderer.h"
#include <cstdio>
#include <cmath>

// 由 eye/target 推导 FPS 相机的 yaw/pitch（Camera::forward 公式的逆变换）
static Camera make_camera(const Vector3& eye, const Vector3& target) {
    Camera c;
    c.position = eye;
    const Vector3 d = (target - eye).normalized();
    c.yaw   = std::atan2(d.x, -d.z) * 57.29578f;   // forward.x = sin(yaw)·cos(p)
    c.pitch = std::asin(d.y) * 57.29578f;          // forward.y = sin(pitch)
    return c;
}

// 渲染开/关阴影两帧，返回差分 >0.05 的像素（影子）加权重心（屏幕坐标）
static bool shadow_centroid(SolidMesh& mesh, FrameBuffer& fb, const Camera& cam,
                            float& cx_out, float& cy_out, int& px_count) {
    ClipWindow win(1.0f);
    PointLight light;
    light.position = {0, 5, 0};                    // 正上方光：影子在模型正下方

    mesh.enable_shadows(false);
    mesh.draw(fb, win, light, cam);
    std::vector<float> off(size_t(fb.width()) * fb.height());
    for (int y = 0; y < fb.height(); ++y)
        for (int x = 0; x < fb.width(); ++x) {
            Vector3 c = fb.pixel(x, y);
            off[size_t(y) * fb.width() + x] = (c.x + c.y + c.z) / 3.0f;
        }

    mesh.enable_shadows(true);
    mesh.draw(fb, win, light, cam);

    double sw = 0, sx = 0, sy = 0;
    int n = 0;
    for (int y = 0; y < fb.height(); ++y)
        for (int x = 0; x < fb.width(); ++x) {
            Vector3 c = fb.pixel(x, y);
            const float d = off[size_t(y) * fb.width() + x] - (c.x + c.y + c.z) / 3.0f;
            if (d > 0.05f) { sw += d; sx += d * x; sy += d * y; ++n; }
        }
    px_count = n;
    if (n == 0) return false;
    cx_out = float(sx / sw);
    cy_out = float(sy / sw);
    return true;
}

int main() {
    int errors = 0;
    auto check = [&](bool ok, const char* what) {
        if (!ok) { std::printf("FAIL: %s\n", what); ++errors; }
    };

    // 地面平面高度：从实际装配的场景里取地面顶点的最高点
    // （模型顶点在前 [0, model_vert_count)，地面顶点在后）
    float ground_top = -1e9f;
    const std::vector<Vector3>& V = SolidMesh::verts();
    for (size_t i = size_t(SolidMesh::model_vert_count()); i < V.size(); ++i)
        ground_top = std::max(ground_top, V[i].y);
    std::printf("ground plane y = %.3f (from_obj=%d, model faces=%d)\n",
                ground_top, int(SolidMesh::loaded_from_obj()), SolidMesh::model_face_count());

    // 两台都不瞄准世界原点的相机——旧代码的视线基用的是"指向原点"的方向，
    // 只有这种机位才能暴露阴影游动（瞄准原点的机位两方向重合，bug 隐形）
    struct Case { const char* name; Vector3 eye, target; };
    const Case cases[] = {
        {"cam A (远机位)", Vector3( 8, 3, 12), Vector3(1.5f, ground_top, -1.0f)},
        {"cam B (另一侧)", Vector3(-5, 2,  7), Vector3(2.2f, ground_top, -1.2f)},
    };

    // 屏幕重心 → 像素射线 → 与地面平面求交 → 世界坐标（重心的世界位置）
    const auto unproject_to_ground = [&](const Camera& cam, float cx, float cy) -> Vector3 {
        const float ndc_x = cx / 300.0f * 2.0f - 1.0f;
        const float ndc_y = 1.0f - cy / 300.0f * 2.0f;
        const Vector3 fwd   = cam.forward();
        const Vector3 right = fwd.cross(Vector3(0, 1.0f, 0)).normalized();
        const Vector3 up    = right.cross(fwd).normalized();
        const float tan_h = std::tan(30.0f * 3.14159265f / 180.0f);  // fov 60° 的一半
        const Vector3 dir = (fwd + right * (ndc_x * tan_h) + up * (ndc_y * tan_h)).normalized();
        const float t = (ground_top - cam.position.y) / dir.y;       // dir.y 必为负（俯视）
        return cam.position + dir * t;
    };

    FrameBuffer fb(300, 300);
    SolidMesh mesh;
    Vector3 world[2];
    for (int ci = 0; ci < 2; ++ci) {
        const Camera cam = make_camera(cases[ci].eye, cases[ci].target);
        float cx = 0, cy = 0;
        int n = 0;
        const bool got = shadow_centroid(mesh, fb, cam, cx, cy, n);
        check(got && n > 300, "影子应覆盖成片像素");
        if (!got) return 1;
        world[ci] = unproject_to_ground(cam, cx, cy);
        std::printf("%s: shadow px=%d centroid=(%.1f, %.1f) -> world (%.3f, %.3f)\n",
                    cases[ci].name, n, cx, cy, world[ci].x, world[ci].z);
    }

    // 核心断言：两台相机看到的影子重心必须是同一个世界点。
    // 视线基错误时反投影结果会差出几个世界单位，容差 0.25 足以区分。
    const float dx = world[0].x - world[1].x, dz = world[0].z - world[1].z;
    const float dist = std::sqrt(dx * dx + dz * dz);
    std::printf("shadow centroid world-distance between cameras = %.3f\n", dist);
    check(dist < 0.25f, "影子重心的世界位置不得随视点改变（阴影不游动）");

    if (errors == 0) std::printf("shadow view-stability regression test: PASS\n");
    return errors == 0 ? 0 : 1;
}
