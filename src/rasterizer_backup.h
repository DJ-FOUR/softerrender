#pragma once
// ============================================================
// 【归档】上一阶段：三角形光栅化 + Blinn-Phong + z-buffer + 自动旋转
//
// 本文件不被任何编译单元包含，仅作历史备份（项目未用 git）。
// 后续按学习路线恢复光栅化时，以此为基础补齐：
//   - OBJ/MTL/纹理加载、透视校正插值（修复旧版贴图"游动"）
//   - Sutherland-Hodgman 视锥体裁剪（替换这里的整三角形丢弃）
//   - 阴影贴图 pass
// ============================================================
// ============================================================
// 软件渲染器（纯 CPU）
// 全部渲染计算在本文件完成，输出一张 RGB 浮点帧缓冲；
// main.cpp 用 OpenGL 的 GL_POINTS 把帧缓冲画到屏幕上（仅此而已）。
//
// 管线阶段（一帧的执行顺序）：
//   1. 顶点变换    model → view → projection → 裁剪空间
//   2. 齐次除法    裁剪空间 → NDC
//   3. 视口变换    NDC → 屏幕坐标（y 翻转）
//   4. 背面剔除    面法线 · 视线方向
//   5. 光栅化      包围盒 + 重心坐标逐像素覆盖测试
//   6. 逐像素      深度测试（消隐）→ 着色（Blinn-Phong）
//   7. 写入帧缓冲
// ============================================================
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include "linalg.h"

// 帧缓冲：一维数组（idx = y*width + x），缓存友好。
// 旧项目的 vector<vector<float>> 每行一次堆间接寻址，这里刻意改进。
struct FrameBuffer {
    int width, height;
    std::vector<float> r, g, b, depth;

    FrameBuffer(int w, int h)
        : width(w), height(h),
          r(size_t(w) * h, 1.0f), g(size_t(w) * h, 1.0f), b(size_t(w) * h, 1.0f),
          depth(size_t(w) * h, 1e6f) {}

    void clear() {
        std::fill(r.begin(), r.end(), 1.0f);      // 背景白色
        std::fill(g.begin(), g.end(), 1.0f);
        std::fill(b.begin(), b.end(), 1.0f);
        std::fill(depth.begin(), depth.end(), 1e6f);
    }
};

// 屏幕空间顶点：像素坐标 + [0,1] 深度 + clip w（透视校正插值时要用）
struct ScreenVert {
    float x, y;    // 屏幕坐标（像素）
    float z;       // NDC 深度映射到 [0,1]
    float w;       // 裁剪空间 w（= -z_view），透视校正插值的权重来源
};

// 重心坐标：返回点 (px,py) 相对三角形三个顶点的权重，
// 三者均 >= 0 表示点在三角形内。原理：面积比 / edge function。
inline bool barycentric(const ScreenVert& v0, const ScreenVert& v1, const ScreenVert& v2,
                        float px, float py, float& a, float& b, float& c) {
    float denom = (v1.y - v2.y) * (v0.x - v2.x) + (v2.x - v1.x) * (v0.y - v2.y);
    if (std::fabs(denom) < 1e-6f) return false;   // 退化三角形
    a = ((v1.y - v2.y) * (px - v2.x) + (v2.x - v1.x) * (py - v2.y)) / denom;
    b = ((v2.y - v0.y) * (px - v2.x) + (v0.x - v2.x) * (py - v2.y)) / denom;
    c = 1.0f - a - b;
    return a >= 0 && b >= 0 && c >= 0;
}

// Blinn-Phong 光照：ambient + diffuse(N·L) + specular(N·H)^s
// H = normalize(L + V) 为半角向量，比 Phong 的反射向量便宜且高光更合理
inline Vector3 blinn_phong(const Vector3& pos, const Vector3& normal,
                           const Vector3& albedo, const Vector3& light_pos,
                           const Vector3& eye) {
    Vector3 ambient(0.18f, 0.18f, 0.18f);
    Vector3 light_color(1.0f, 1.0f, 1.0f);

    Vector3 L = (light_pos - pos).normalized();
    Vector3 V = (eye - pos).normalized();
    Vector3 H = (L + V).normalized();

    float ndl = std::max(0.0f, normal.dot(L));
    float spec = 0.0f;
    if (ndl > 0.0f) spec = std::pow(std::max(0.0f, normal.dot(H)), 32.0f);

    Vector3 color = albedo * (ambient.x + light_color.x * ndl) +
                    light_color * (0.6f * spec);
    return {std::min(1.0f, color.x), std::min(1.0f, color.y), std::min(1.0f, color.z)};
}

// 渲染一个绕 Y 轴旋转的彩色立方体 —— 完整走一遍管线
inline void render_scene(FrameBuffer& fb, float angle_deg) {
    fb.clear();

    // ---------- 管线 1：矩阵准备 ----------
    const Vector3 eye(0, 1.2f, 4.0f);                 // 相机位置
    Mat4 model = Mat4::rotate_y(angle_deg);           // 模型变换（旋转）
    Mat4 view  = Mat4::look_at(eye, Vector3(0, 0, 0), Vector3(0, 1, 0));
    Mat4 proj  = Mat4::perspective(60.0f, fb.width / float(fb.height), 0.1f, 100.0f);
    Mat4 vp    = Mat4::multiply(proj, view);          // 通常再乘 model，这里模型只有旋转

    // 立方体：8 个顶点、6 个面（每面拆 2 个三角形，从外看逆时针 = 正面）
    static const Vector3 cube_v[8] = {
        {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
        {-1, -1,  1}, {1, -1,  1}, {1, 1,  1}, {-1, 1,  1},
    };
    static const int cube_f[12][3] = {
        {4, 5, 6}, {4, 6, 7},    // 前 (+z) 蓝
        {1, 0, 3}, {1, 3, 2},    // 后 (-z) 青
        {5, 1, 2}, {5, 2, 6},    // 右 (+x) 橙
        {0, 4, 7}, {0, 7, 3},    // 左 (-x) 紫
        {3, 7, 6}, {3, 6, 2},    // 顶 (+y) 黄绿
        {4, 0, 1}, {4, 1, 5},    // 底 (-y) 红
    };
    static const Vector3 face_color[6] = {
        {0.25f, 0.45f, 0.90f}, {0.20f, 0.75f, 0.70f},
        {0.90f, 0.50f, 0.20f}, {0.60f, 0.30f, 0.80f},
        {0.70f, 0.80f, 0.20f}, {0.85f, 0.25f, 0.25f},
    };

    const Vector3 light_pos(2.0f, 3.0f, 2.5f);

    for (int f = 0; f < 12; ++f) {
        // ---------- 管线 1：顶点变换 model → world ----------
        Vector3 world[3];
        for (int i = 0; i < 3; ++i) world[i] = model.transform_point(cube_v[cube_f[f][i]]);

        // 面法线（世界空间）：叉积得到，注意绕序决定朝向
        Vector3 normal = (world[1] - world[0]).cross(world[2] - world[0]).normalized();

        // ---------- 管线 4：背面剔除 ----------
        // 从外看逆时针的面为正面：法线指向观察者则可见
        if (normal.dot(eye - world[0]) <= 0.0f) continue;

        // ---------- 管线 1-3：投影 → 齐次除法 → 视口变换 ----------
        ScreenVert sv[3];
        bool clipped = false;
        for (int i = 0; i < 3; ++i) {
            Vec4 clip = vp.transform_point_clip(world[i]);
            if (clip.w < 0.05f) { clipped = true; break; }  // 简化近平面保护
            float inv_w = 1.0f / clip.w;                    // 齐次除法
            float ndc_x = clip.x * inv_w, ndc_y = clip.y * inv_w, ndc_z = clip.z * inv_w;
            sv[i].x = (ndc_x + 1.0f) * 0.5f * fb.width;    // 视口变换
            sv[i].y = (1.0f - ndc_y) * 0.5f * fb.height;   // y 翻转（图像坐标向下）
            sv[i].z = (ndc_z + 1.0f) * 0.5f;               // 深度 [0,1]
            sv[i].w = clip.w;                               // 透视校正插值要用
        }
        if (clipped) continue;
        // 注：真正的管线应在这里对视锥体做 Sutherland-Hodgman 裁剪，
        // 上面只是整三角形丢弃的简化版（旧项目完全没做这一步）。

        // ---------- 管线 5：包围盒 ----------
        float min_x = std::min({sv[0].x, sv[1].x, sv[2].x});
        float max_x = std::max({sv[0].x, sv[1].x, sv[2].x});
        float min_y = std::min({sv[0].y, sv[1].y, sv[2].y});
        float max_y = std::max({sv[0].y, sv[1].y, sv[2].y});
        int x0 = std::max(0, int(std::floor(min_x)));
        int x1 = std::min(fb.width - 1, int(std::ceil(max_x)));
        int y0 = std::max(0, int(std::floor(min_y)));
        int y1 = std::min(fb.height - 1, int(std::ceil(max_y)));

        // 每个三角形自己的材质颜色（演示用）
        const Vector3& albedo = face_color[f / 2];

        // ---------- 管线 5-7：逐像素光栅化 + 深度测试 + 着色 ----------
        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                float a, b, c;
                if (!barycentric(sv[0], sv[1], sv[2], x + 0.5f, y + 0.5f, a, b, c))
                    continue;   // 像素中心不在三角形内

                // NDC 深度在屏幕空间是仿射的，可直接线性插值（z-buffer 消隐）
                float z = a * sv[0].z + b * sv[1].z + c * sv[2].z;
                int idx = y * fb.width + x;
                if (z > fb.depth[idx]) continue;            // 深度测试：被遮挡
                fb.depth[idx] = z;

                // 着色：本演示用面法线做平直着色（flat shading）。
                // 若有逐顶点属性（颜色/UV/法线），必须用透视校正插值：
                //   attr = Σ(aᵢ·attrᵢ/wᵢ) / Σ(aᵢ/wᵢ)   ← 旧项目此处的 bug 来源
                Vector3 centroid = (world[0] + world[1] + world[2]) * (1.0f / 3.0f);
                Vector3 color = blinn_phong(centroid, normal, albedo, light_pos, eye);

                fb.r[idx] = color.x;
                fb.g[idx] = color.y;
                fb.b[idx] = color.z;
            }
        }
    }
}
