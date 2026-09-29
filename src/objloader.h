#pragma once
// ============================================================
// OBJ 网格加载器（Wavefront .obj，教学向最小实现）
//
// 支持：
//   v / vt / vn            顶点、UV、法线（三套索引彼此独立）
//   f                      四种角点写法：v、v/vt、v//vn、v/vt/vn
//   mtllib                 自动加载 .mtl（相对 .obj 所在目录解析）
//   usemtl                 按面记录材质编号（materials 下标）
//   o / g                  对象分组（faces 里存 object 编号，供场景装配用）
//   s                      平滑标记（按面记录；当前渲染走平面着色）
//
// 规则：
//   - 所有索引统一转成 0 基；负数索引按 OBJ 规范解释为"从尾部倒数"；
//   - 多于 3 个顶点的面按扇形拆成三角形（导出前最好已在 Blender
//     里三角面化，这里只是兜底——凹多边形的扇形拆分会有误差）；
//   - 解析器只做忠实转换，不猜语义；"哪个对象当地面"这类场景装配
//     决策在 SolidMesh::build_scene 里做。
// ============================================================
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "linalg.h"
#include "mtlloader.h"

struct ObjFace {
    int v[3];            // 顶点索引（0 基）
    int t[3];            // UV 索引（0 基，-1 = 无）
    int n[3];            // 法线索引（0 基，-1 = 无）
    int material = -1;   // materials 下标（usemtl 之前为 -1）
    int object = -1;     // objects 下标（o/g 之前为 -1）
    bool smooth = false; // s 行的平滑标记
};

struct ObjObject {
    std::string name;
};

struct ObjModel {
    std::vector<Vector3> verts;
    std::vector<Vector3> uvs;       // 只用 x/y（z 恒 0），贴图阶段接入
    std::vector<Vector3> normals;
    std::vector<ObjFace> faces;     // 全部三角化
    std::vector<Material> materials;
    std::vector<ObjObject> objects;
    bool ok = false;                // 是否解析出可用几何
};

// "v/vt/vn" 角点 → 三个 0 基索引；负数 = 从尾部倒数；缺省段 = -1
inline void parse_obj_corner(const std::string& tok, size_t nv, size_t nt, size_t nn,
                             int out[3]) {
    const auto to_index = [](const std::string& s, size_t count) -> int {
        if (s.empty()) return -1;
        const int i = std::atoi(s.c_str());
        return i > 0 ? i - 1 : (i < 0 ? int(count) + i : -1);
    };
    out[0] = out[1] = out[2] = -1;
    const size_t s1 = tok.find('/');
    if (s1 == std::string::npos) {                 // "v"
        out[0] = to_index(tok, nv);
        return;
    }
    out[0] = to_index(tok.substr(0, s1), nv);
    const size_t s2 = tok.find('/', s1 + 1);
    if (s2 == std::string::npos) {                 // "v/vt"
        out[1] = to_index(tok.substr(s1 + 1), nt);
        return;
    }
    out[1] = to_index(tok.substr(s1 + 1, s2 - s1 - 1), nt);   // "v/vt/vn" 或 "v//vn"
    out[2] = to_index(tok.substr(s2 + 1), nn);
}

// 读取一个 .obj；失败时返回 ok=false，原因写入 *err（可空）。
inline bool load_obj(const std::string& path, ObjModel& out, std::string* err = nullptr) {
    out = ObjModel{};
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        if (err) *err = "打不开 " + path;
        return false;
    }
    const std::string dir = dirname_of(path);

    int cur_mat = -1, cur_obj = -1;
    bool cur_smooth = false;
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();   // 兼容 CRLF
        std::istringstream ss(line);
        std::string tag;
        ss >> tag;
        if (tag == "v" || tag == "vt" || tag == "vn") {
            Vector3 p;
            ss >> p.x >> p.y >> p.z;
            if (tag == "v") out.verts.push_back(p);
            else if (tag == "vt") out.uvs.push_back(p);
            else out.normals.push_back(p);
        } else if (tag == "f") {
            std::vector<std::string> toks;
            for (std::string tok; ss >> tok;) toks.push_back(tok);
            if (toks.size() < 3) continue;                 // 坏行防御
            const size_t nv = out.verts.size(), nt = out.uvs.size(), nn = out.normals.size();
            int c0[3];
            parse_obj_corner(toks[0], nv, nt, nn, c0);
            for (size_t i = 1; i + 1 < toks.size(); ++i) { // 扇形三角化：v0,vi,vi+1
                int c1[3], c2[3];
                parse_obj_corner(toks[i], nv, nt, nn, c1);
                parse_obj_corner(toks[i + 1], nv, nt, nn, c2);
                ObjFace f;
                for (int k = 0; k < 3; ++k) {
                    f.v[k] = (k == 0) ? c0[0] : (k == 1) ? c1[0] : c2[0];
                    f.t[k] = (k == 0) ? c0[1] : (k == 1) ? c1[1] : c2[1];
                    f.n[k] = (k == 0) ? c0[2] : (k == 1) ? c1[2] : c2[2];
                }
                f.material = cur_mat;
                f.object = cur_obj;
                f.smooth = cur_smooth;
                out.faces.push_back(f);
            }
        } else if (tag == "mtllib") {
            std::string rest;
            std::getline(ss, rest);
            const std::string lib = trim_string(rest);
            if (lib.empty()) continue;
            // 绝对路径（D:/ 或 / 开头）直接用；相对路径相对 .obj 所在目录
            const bool absolute = (lib.size() > 1 && lib[1] == ':') || lib[0] == '/';
            std::vector<Material> loaded = load_mtl(absolute ? lib : dir + lib);
            out.materials.insert(out.materials.end(), loaded.begin(), loaded.end());
        } else if (tag == "usemtl") {
            std::string name;
            ss >> name;
            cur_mat = -1;                                  // 找不到就按无材质渲染
            for (size_t i = 0; i < out.materials.size(); ++i)
                if (out.materials[i].name == name) { cur_mat = int(i); break; }
        } else if (tag == "o" || tag == "g") {
            std::string name;
            ss >> name;
            out.objects.push_back({name});
            cur_obj = int(out.objects.size()) - 1;
        } else if (tag == "s") {
            std::string v;
            ss >> v;
            cur_smooth = (v != "0" && v != "off");
        }
        // 其余标签（#、l、usemap…）一律忽略
    }
    out.ok = !out.verts.empty() && !out.faces.empty();
    if (!out.ok && err) *err = path + " 里没有可用的 v/f 数据";
    return out.ok;
}
