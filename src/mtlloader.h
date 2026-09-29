#pragma once
// ============================================================
// MTL 材质库加载器（Wavefront .mtl，教学向最小实现）
//
// 只解析渲染用得上的字段：
//   Kd  漫反射反照率（规范缺省 0.8 灰；Blender 导出时常不写 Kd、
//       把颜色放进 map_Kd 纹理——贴图阶段接入前模型会是灰色）
//   Ks  镜面反射率（与光源镜面强度相乘后进 Blinn-Phong 高光项）
//   Ns  高光指数（shininess）
//   map_Kd 漫反射纹理路径——本阶段只解析并回退定位到可用路径，
//          不读图像内容（stb_image + UV 采样属于下一阶段）
// 其余字段（Ka/Ke/Ni/d/Tr/illum...）读到了也先忽略。
// ============================================================
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "linalg.h"

struct Material {
    std::string name;
    Vector3 kd{0.8f, 0.8f, 0.8f};   // 漫反射反照率
    Vector3 ks{0.0f, 0.0f, 0.0f};   // 镜面反射率
    float shininess = 32.0f;        // Ns：高光指数
    bool has_kd = false;            // 区分"文件没写 Kd"与"写了 0.8 灰"
    std::string map_kd;             // 漫反射纹理路径（已尽力解析为可用路径，可空）
};

// 路径工具：取目录部分（含结尾分隔符；无分隔符返回空串）
inline std::string dirname_of(const std::string& path) {
    const size_t p = path.find_last_of("/\\");
    return (p == std::string::npos) ? std::string() : path.substr(0, p + 1);
}

inline bool file_exists(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    return f.good();
}

inline std::string trim_string(const std::string& s) {
    const size_t b = s.find_first_not_of(" \t");
    if (b == std::string::npos) return {};
    const size_t e = s.find_last_not_of(" \t");
    return s.substr(b, e - b + 1);
}

// map_Kd 常见写不通的情况：路径来自建模机的绝对路径（如 D:/PC/Pictures/...），
// 换机器后失效。回退策略：按原路径找不到时，取文件名部分到 .mtl 同目录找
// （Blender 导出的纹理通常就躺在 .mtl 旁边）。
inline std::string resolve_texture_path(const std::string& raw, const std::string& mtl_dir) {
    if (raw.empty()) return {};
    if (file_exists(raw)) return raw;
    const size_t p = raw.find_last_of("/\\");
    const std::string base = (p == std::string::npos) ? raw : raw.substr(p + 1);
    const std::string cand = mtl_dir + base;
    return file_exists(cand) ? cand : raw;
}

// 读取一个 .mtl（一个文件可含多个 newmtl）；打不开时返回空表。
inline std::vector<Material> load_mtl(const std::string& path) {
    std::vector<Material> mats;
    std::ifstream in(path, std::ios::binary);
    if (!in) return mats;
    const std::string dir = dirname_of(path);

    Material cur;
    bool open = false;
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();   // 兼容 CRLF
        std::istringstream ss(line);
        std::string tag;
        ss >> tag;
        if (tag == "newmtl") {
            if (open) mats.push_back(cur);
            cur = Material();
            ss >> cur.name;              // 材质名按字节原样保存（可能是 UTF-8，如"材质"）
            open = true;
        } else if (!open) {
            continue;                    // 第一个 newmtl 之前的行没有归属
        } else if (tag == "Kd") {
            ss >> cur.kd.x >> cur.kd.y >> cur.kd.z;
            cur.has_kd = true;
        } else if (tag == "Ks") {
            ss >> cur.ks.x >> cur.ks.y >> cur.ks.z;
        } else if (tag == "Ns") {
            ss >> cur.shininess;
        } else if (tag == "map_Kd") {
            std::string rest;
            std::getline(ss, rest);      // 路径可能含空格，取 tag 后整段
            cur.map_kd = resolve_texture_path(trim_string(rest), dir);
        }
        // d/Tr/Ka/Ke/Ni/illum 等其余字段忽略
    }
    if (open) mats.push_back(cur);
    return mats;
}
