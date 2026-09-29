#pragma once
// ============================================================
// 纹理：加载（stb_image 单头库）+ 双线性采样。
//
// 采样约定：
//   - OBJ 的 UV 原点在图像左下角（v=0 是图像最后一行），而图像数据
//     第一行在顶部——采样时对 v 做翻转；
//   - u/v 超出 [0,1] 按 repeat 回绕（floor 取小数部分）；
//     纹素越界按 clamp-to-edge 钳制；
//   - 返回线性域 [0,1]^3 的 RGB（与 FrameBuffer 的浮点颜色一致）。
//
// 本项目每个可执行文件只有一个翻译单元（main.cpp 或单个测试），
// 在这里一次性展开 stb_image 的实现即可。
// ============================================================
#ifndef STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#endif
#include "stb_image.h"
#include <algorithm>
#include <string>
#include <vector>
#include "linalg.h"

struct Texture {
    int width = 0, height = 0;
    std::vector<unsigned char> rgb;   // 3 通道 8bit，行主序，(0,0) 在左上；空 = 无纹理
    bool ok() const { return width > 0 && height > 0 && !rgb.empty(); }
};

// 读入一张图片并强制转成 3 通道 RGB；失败返回 width=0 的空纹理
inline Texture load_texture(const std::string& path) {
    Texture t;
    int n = 0;
    unsigned char* data = stbi_load(path.c_str(), &t.width, &t.height, &n, 3);
    if (!data) return t;
    t.rgb.assign(data, data + size_t(t.width) * size_t(t.height) * 3);
    stbi_image_free(data);
    return t;
}

// 双线性采样。u/v 为任意实数（会回绕），返回 [0,1]^3 的 RGB。
inline Vector3 sample_texture(const Texture& t, float u, float v) {
    if (!t.ok()) return {1, 1, 1};
    u -= std::floor(u);                       // repeat 回绕
    v -= std::floor(v);
    const float fx = u * float(t.width) - 0.5f;             // 像素中心对齐
    const float fy = (1.0f - v) * float(t.height) - 0.5f;   // OBJ 的 v 轴从底部起
    const int x0 = int(std::floor(fx)), y0 = int(std::floor(fy));
    const float tx = fx - float(x0), ty = fy - float(y0);
    const auto fetch = [&](int xi, int yi) -> Vector3 {
        xi = std::min(std::max(xi, 0), t.width - 1);        // clamp-to-edge
        yi = std::min(std::max(yi, 0), t.height - 1);
        const size_t i = (size_t(yi) * size_t(t.width) + size_t(xi)) * 3;
        return {t.rgb[i] / 255.0f, t.rgb[i + 1] / 255.0f, t.rgb[i + 2] / 255.0f};
    };
    return (fetch(x0, y0) * (1.0f - tx) + fetch(x0 + 1, y0) * tx) * (1.0f - ty) +
           (fetch(x0, y0 + 1) * (1.0f - tx) + fetch(x0 + 1, y0 + 1) * tx) * ty;
}
