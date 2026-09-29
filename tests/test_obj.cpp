// OBJ/MTL 加载器回归测试。
// 编译运行（不在 CMake 里，手动跑）：
//   D:/mingw64/bin/g++.exe -std=c++17 -Isrc tests/test_obj.cpp -o build/test_obj.exe
//   ./build/test_obj.exe
#include "objloader.h"
#include <cstdio>
#include <cmath>
#include <fstream>

int main() {
    int errors = 0;
    auto check = [&](bool ok, const char* what) {
        if (!ok) { std::printf("FAIL: %s\n", what); ++errors; }
    };

    // ========== ① 合成用例：负数索引 + 四边面扇形三角化 + 缺省法线 ==========
    {
        std::ofstream tmp("test_tmp_quad.obj", std::ios::binary);
        tmp << "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\n"
            << "f -4 -3 -2 -1\n"           // 负索引=从尾部倒数 → (0,1,2),(0,2,3)
            << "f 1 2 3\n";                // 正索引 1 基 → (0,1,2)
        tmp.close();

        ObjModel m;
        std::string err;
        const bool ok = load_obj("test_tmp_quad.obj", m, &err);
        check(ok, "合成 OBJ 应能打开");
        check(m.verts.size() == 4, "合成 OBJ 顶点数应为 4");
        check(m.faces.size() == 3, "四边面扇形拆 2 个三角形 + 1 个三角形 = 3 面");
        if (m.faces.size() == 3) {
            check(m.faces[0].v[0] == 0 && m.faces[0].v[1] == 1 && m.faces[0].v[2] == 2,
                  "负索引扇形第一片应为 (0,1,2)");
            check(m.faces[1].v[0] == 0 && m.faces[1].v[1] == 2 && m.faces[1].v[2] == 3,
                  "负索引扇形第二片应为 (0,2,3)");
            check(m.faces[2].v[0] == 0 && m.faces[2].v[1] == 1 && m.faces[2].v[2] == 2,
                  "1 基正索引应转为 0 基 (0,1,2)");
            check(m.faces[0].n[0] == -1, "缺省 vn 段应为 -1");
        }
        std::remove("test_tmp_quad.obj");
    }

    // ========== ② 真实资产：ball.obj + ball.mtl（模型资产在 src/ 下）==========
    const char* paths[] = {"src/ball.obj", "../src/ball.obj",
                           "asset/ball.obj", "../asset/ball.obj"};
    ObjModel model;
    bool loaded = false;
    for (const char* p : paths) {
        std::string err;
        if (load_obj(p, model, &err)) { loaded = true; break; }
    }
    if (!loaded) {
        std::printf("SKIP: 找不到 src/ball.obj（其余断言基于真实资产，跳过）\n");
        return errors == 0 ? 0 : 1;
    }
    std::printf("ball.obj: %zu verts, %zu uvs, %zu normals, %zu faces, %zu materials, %zu objects\n",
                model.verts.size(), model.uvs.size(), model.normals.size(),
                model.faces.size(), model.materials.size(), model.objects.size());

    check(model.verts.size() == 490, "顶点数应为 490");
    check(model.uvs.size() == 573, "UV 数应为 573");
    check(model.normals.size() == 488, "法线数应为 488（Shade Smooth 后按顶点合并）");
    check(model.faces.size() == 972, "三角面数应为 972（Blender 已三角面化）");
    check(model.materials.size() == 2, "应有 2 个材质（球体贴图 + 地面纯色）");
    check(model.objects.size() == 2, "应解析出 2 个对象（球体 + 立方体）");

    // 所有面都是三角形，且三套索引都不越界
    bool tri_ok = true, idx_ok = true;
    for (const ObjFace& f : model.faces) {
        for (int k = 0; k < 3; ++k) {
            if (f.v[k] < 0 || f.v[k] >= int(model.verts.size())) tri_ok = false;
            if (f.n[k] >= 0 && f.n[k] >= int(model.normals.size())) idx_ok = false;
            if (f.t[k] >= 0 && f.t[k] >= int(model.uvs.size())) idx_ok = false;
        }
        if (f.material >= int(model.materials.size())) idx_ok = false;
        if (f.object >= int(model.objects.size())) idx_ok = false;
    }
    check(tri_ok, "每个面都应是三角形且顶点索引不越界");
    check(idx_ok, "vn/vt/材质/对象索引都不越界");

    // 绕序必须与法线一致（决定背面剔除方向；Blender 导出 CCW 朝外）
    int disagree = 0;
    for (const ObjFace& f : model.faces) {
        if (f.n[0] < 0) continue;
        const Vector3& a = model.verts[size_t(f.v[0])];
        const Vector3& b = model.verts[size_t(f.v[1])];
        const Vector3& c = model.verts[size_t(f.v[2])];
        const Vector3 gn = (b - a).cross(c - a);
        if (gn.dot(model.normals[size_t(f.n[0])]) < 0.0f) ++disagree;
    }
    check(disagree == 0, "所有面的几何法线应与 vn 同向（绕序朝外）");

    // 材质：Ns/Ks/map_Kd 都应读到，map_Kd 应回退解析到实际存在的文件
    const Material* mat = nullptr;
    const Material* mat2 = nullptr;
    for (const Material& m : model.materials) {
        if (m.name == "\xE6\x9D\x90\xE8\xB4\xA8") mat = &m;              // UTF-8 "材质"
        if (m.name == "\xE6\x9D\x90\xE8\xB4\xA8.001") mat2 = &m;         // "材质.001"
    }
    check(mat != nullptr, "应存在名为'材质'的材质（usemtl 与 newmtl 字节匹配）");
    if (mat) {
        std::printf("material: Ns=%.1f Ks=%.2f map_Kd=%s has_kd=%d\n",
                    mat->shininess, mat->ks.x, mat->map_kd.c_str(), int(mat->has_kd));
        check(std::fabs(mat->shininess - 250.0f) < 1e-3f, "Ns 应为 250");
        check(std::fabs(mat->ks.x - 0.5f) < 1e-3f, "Ks 应为 0.5");
        check(!mat->map_kd.empty() && file_exists(mat->map_kd),
              "map_Kd 应解析到实际存在的文件（src/tietu.png）");
        check(!mat->has_kd, "球体材质未写 Kd（颜色全部来自 map_Kd 纹理）");
    }
    check(mat2 != nullptr && mat2->has_kd, "地面材质'材质.001'应带 Kd 纯色");

    // usemtl 覆盖：全部 972 个面都应挂上材质
    bool all_matted = true;
    for (const ObjFace& f : model.faces)
        if (f.material < 0) all_matted = false;
    check(all_matted, "所有面都应挂上材质编号");

    if (errors == 0) std::printf("obj/mtl loader regression test: PASS\n");
    return errors == 0 ? 0 : 1;
}
