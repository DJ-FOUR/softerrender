// 阴影链路诊断：光源深度图 → 已知阴影点的重投影 → 逐步对照。
// 编译运行：
//   D:/mingw64/bin/g++.exe -std=c++17 -Isrc tests/debug_shadow.cpp -o build/debug_shadow.exe
//   ./build/debug_shadow.exe
#include "soft_renderer.h"
#include <cstdio>
#include <fstream>

static Vector3 mesh_vertex(int ix) {
    // 与 SolidMesh::kVertices 一致的硬编码副本（调试用）
    static const Vector3 v[20] = {
        {-1.5f, -1.5f,  1}, { 1.5f, -1.5f,  1}, { 1.5f, -0.5f,  1}, { 0.5f, -0.5f,  1},
        { 0.5f,  0.5f,  1}, {-0.5f,  0.5f,  1}, {-0.5f,  1.5f,  1}, {-1.5f,  1.5f,  1},
        {-1.5f, -1.5f, -1}, { 1.5f, -1.5f, -1}, { 1.5f, -0.5f, -1}, { 0.5f, -0.5f, -1},
        { 0.5f,  0.5f, -1}, {-0.5f,  0.5f, -1}, {-0.5f,  1.5f, -1}, {-1.5f,  1.5f, -1},
        {-6, -1.52f, -6}, {-6, -1.52f, 6}, {6, -1.52f, 6}, {6, -1.52f, -6},
    };
    return v[ix];
}

int main() {
    std::string NL;  NL += char(92);  NL += 'n';   // C++ 换行转义序列
    SolidMesh mesh;
    PointLight light;  light.position = {1.2f, 3, -15};

    // ---- ① 光源深度图 ----
    FrameBuffer smap(512, 512);
    Mat4 lview = Mat4::look_at(light.position, Vector3(0, 0, 0), Vector3(0, 1, 0));
    Mat4 lproj = Mat4::perspective(100.0f, 1.0f, 0.1f, 60.0f);
    Mat4 lvp = lproj * lview;
    mesh.render_depth(smap, lvp, Mat4());   // 恒等模型矩阵（debug 静态场景）

    int filled = 0;
    float dmin = 1e9f, dmax = -1e9f;
    for (int y = 0; y < 512; ++y)
        for (int x = 0; x < 512; ++x) {
            float d = smap.depth_at(x, y);
            if (d < 1.0f) { ++filled; dmin = std::fmin(dmin, d); dmax = std::fmax(dmax, d); }
        }
    std::printf("shadow map: filled=%d/262144  dmin=%.4f dmax=%.4f\n", filled, dmin, dmax);
    std::printf("center(256,256) depth=%.4f\n", smap.depth_at(256, 256));

    // ---- ② 取一个理应被阴影覆盖的地面点，手动走重投影链路 ----
    // 光源 (4,1,0)，点 (−2,−1.52,0)：光线穿过台阶实体 → 应在阴影中
    Vector3 world(-2, -1.52f, 0);
    Vec4 lc = lvp.transform_point_clip(world);
    float inv = 1.0f / lc.w;
    int su = int(((lc.x * inv) + 1.0f) * 0.5f * 512);
    int sv = int((1.0f - (lc.y * inv)) * 0.5f * 512);
    float my = (lc.z * inv + 1.0f) * 0.5f;
    std::printf("\nshadow-test point: lc=(%.3f, %.3f, %.3f) w=%.3f\n", lc.x, lc.y, lc.z, lc.w);
    std::printf("  uv=(%d,%d)  my_depth=%.4f  stored=%.4f  (bias=0.0015)\n",
                su, sv, my, smap.depth_at(su, sv));
    std::printf("  => %s\n",
                (my > smap.depth_at(su, sv) + 0.0015f) ? "IN SHADOW (correct)"
                                                       : "LIT (wrong!)");

    // ---- ③ 对照：一个理应被照亮的面（踏步3 顶面朝上，光源在侧上方）----
    Vector3 world2(-1, 1.5f, 0);   // 踏步3 顶面上的一点
    Vec4 lc2 = lvp.transform_point_clip(world2);
    float inv2 = 1.0f / lc2.w;
    int su2 = int(((lc2.x * inv2) + 1.0f) * 0.5f * 512);
    int sv2 = int((1.0f - (lc2.y * inv2)) * 0.5f * 512);
    float my2 = (lc2.z * inv2 + 1.0f) * 0.5f;
    std::printf("\nlit-test point: lc w=%.3f uv=(%d,%d) my=%.4f stored=%.4f => %s\n",
                lc2.w, su2, sv2, my2, smap.depth_at(su2, sv2),
                (my2 > smap.depth_at(su2, sv2) + 0.0015f) ? "shadow" : "lit");
    // ---- ④ 完整 draw() 的阴影开/关对比 ----
    ClipWindow win;
    FrameBuffer fb(300, 300);
    Camera cam;
    mesh.draw(fb, win, light, cam);
    std::vector<float> on_px(300 * 300);
    for (int y = 0; y < 300; ++y)
        for (int x = 0; x < 300; ++x) {
            Vector3 c = fb.pixel(x, y);
            on_px[y * 300 + x] = (c.x + c.y + c.z) / 3.0f;
        }
    mesh.enable_shadows(false);
    mesh.draw(fb, win, light);
    int diff = 0, fx = -1, fy = -1;
    float off_first = 0.0f;
    for (int y = 0; y < 300; ++y)
        for (int x = 0; x < 300; ++x) {
            Vector3 c = fb.pixel(x, y);
            float b = (c.x + c.y + c.z) / 3.0f;
            if (std::fabs(b - on_px[y * 300 + x]) > 1e-4f) {
                ++diff;
                if (fx < 0) { fx = x; fy = y; off_first = b; }
            }
        }
    mesh.enable_shadows(true);
    std::printf("draw() on/off diff pixels: %d", diff);
    if (fx >= 0)
        std::printf("  first at (%d,%d) on=%.3f off=%.3f", fx, fy, on_px[fy * 300 + fx], off_first);
    std::printf("%s", NL.c_str());

    // ---- ⑤ 探针：地面像素手动复现 draw() 着色器的重建与采样 ----
    int px = 60, py = 260;
    float z01 = fb.depth_at(px, py);
    Mat4 view = Mat4::look_at(Vector3(0, 1.8f, 4.5f), Vector3(0, 0, 0), Vector3(0, 1, 0));
    Mat4 proj = Mat4::perspective(60.0f, 1.0f, 0.1f, 100.0f);
    float kC = proj.m[10], kE = proj.m[11];
    float ndc_x = px / 300.0f * 2.0f - 1.0f;
    float ndc_y = 1.0f - py / 300.0f * 2.0f;
    float w = kE / (2.0f * z01 - 1.0f + kC);
    Vector3 fwd = (Vector3(0, 0, 0) - Vector3(0, 1.8f, 4.5f)).normalized();
    Vector3 rgt = fwd.cross(Vector3(0, 1, 0)).normalized();
    Vector3 upv = rgt.cross(fwd).normalized();
    Vector3 pworld = Vector3(0, 1.8f, 4.5f) + rgt * (ndc_x * w / proj.m[0])
                  + upv * (ndc_y * w / proj.m[5]) + fwd * w;
    std::printf("probe (%d,%d): z01=%.4f w=%.3f pworld=(%.2f,%.2f,%.2f)%s",
                px, py, z01, w, world.x, world.y, world.z, NL.c_str());
    Vec4 plc = lvp.transform_point_clip(pworld);
    float pinv = 1.0f / plc.w;
    int pu = int(((plc.x * pinv) + 1.0f) * 0.5f * 512);
    int pv = int((1.0f - (plc.y * pinv)) * 0.5f * 512);
    float pmy = (plc.z * pinv + 1.0f) * 0.5f;
    std::printf("  light uv=(%d,%d) my=%.4f stored=%.4f => %s%s",
                pu, pv, pmy, smap.depth_at(pu, pv),
                (pmy > smap.depth_at(pu, pv) + 0.0015f) ? "SHADOW" : "lit", NL.c_str());

    // ---- ⑤b 近处地面一行像素的"片元光源深度 vs 图中存储深度"对照 ----
    {
        Mat4 view = Mat4::look_at(Vector3(0, 1.8f, 4.5f), Vector3(0, 0, 0), Vector3(0, 1, 0));
        Mat4 proj = Mat4::perspective(60.0f, 1.0f, 0.1f, 100.0f);
        float kC = proj.m[10], kE = proj.m[11];
        Vector3 fwd = (Vector3(0, 0, 0) - Vector3(0, 1.8f, 4.5f)).normalized();
        Vector3 rgt = fwd.cross(Vector3(0, 1, 0)).normalized();
        Vector3 upv = rgt.cross(fwd).normalized();
        for (int py2 = 250; py2 <= 280; py2 += 6) {
            int px2 = 150;
            float dz = fb.depth_at(px2, py2);
            if (dz >= 1.0e6f) { std::printf("y=%d background%s", py2, NL.c_str()); continue; }
            float nx = px2 / 300.0f * 2.0f - 1.0f;
            float ny = 1.0f - py2 / 300.0f * 2.0f;
            float ww = kE / (2.0f * dz - 1.0f + kC);
            Vector3 wp = Vector3(0, 1.8f, 4.5f) + rgt * (nx * ww / proj.m[0])
                       + upv * (ny * ww / proj.m[5]) + fwd * ww;
            Vec4 lc = lvp.transform_point_clip(wp);
            float inv = 1.0f / lc.w;
            int su = int(((lc.x * inv) + 1.0f) * 0.5f * 512);
            int sv = int((1.0f - (lc.y * inv)) * 0.5f * 512);
            float my = (lc.z * inv + 1.0f) * 0.5f;
            std::printf("y=%d w=%.2f world=(%.2f,%.2f,%.2f) uv=(%d,%d) my=%.4f stored=%.4f my-stored=%+.4f%s",
                        py2, ww, wp.x, wp.y, wp.z, su, sv, my,
                        smap.depth_at(su, sv), my - smap.depth_at(su, sv), NL.c_str());
        }
    }

    // ---- ⑥ 全图扫描：所有"地面且位于影子应在区域（台阶左侧）"的像素，
    // 逐个跑着色器同款重投影，统计本应处于阴影的像素数量 ----
    int ground_n = 0, shadow_n = 0, first_sx = -1, first_sy = 0;
    for (int y = 0; y < 300; ++y)
        for (int x = 0; x < 300; ++x) {
            float dz = fb.depth_at(x, y);
            if (dz >= 1.0e6f) continue;
            float nx = x / 300.0f * 2.0f - 1.0f;
            float ny = 1.0f - y / 300.0f * 2.0f;
            float ww = kE / (2.0f * dz - 1.0f + kC);
            Vector3 wpos = Vector3(0, 1.8f, 4.5f) + rgt * (nx * ww / proj.m[0])
                         + upv * (ny * ww / proj.m[5]) + fwd * ww;
            if (std::fabs(wpos.y + 1.52f) > 0.02f) continue;    // 只看地面像素
            ++ground_n;
            if (wpos.x > -1.6f) continue;                       // 只看台阶左侧
            Vec4 c2 = lvp.transform_point_clip(wpos);
            if (c2.w <= 0.0f) continue;
            float iv = 1.0f / c2.w;
            int uu = int(((c2.x * iv) + 1.0f) * 0.5f * 512);
            int vv = int((1.0f - (c2.y * iv)) * 0.5f * 512);
            if (uu < 0 || uu >= 512 || vv < 0 || vv >= 512) continue;
            float mm = (c2.z * iv + 1.0f) * 0.5f;
            if (mm > smap.depth_at(uu, vv) + 0.0015f) {
                ++shadow_n;
                if (first_sx < 0) { first_sx = x; first_sy = y; }
            }
        }
    std::printf("ground pixels=%d  should-be-shadow=%d", ground_n, shadow_n);
    if (first_sx >= 0) std::printf("  first at (%d,%d)", first_sx, first_sy);
    std::printf("%s", NL.c_str());

    // ---- ⑦ 底半幅像素普查：深度/颜色采样，看地面到底渲染了没有 ----
    int nonbg = 0;
    for (int y = 200; y < 300; ++y)
        for (int x = 0; x < 300; ++x)
            if (fb.depth_at(x, y) < 1.0e6f) ++nonbg;
    std::printf("bottom-half non-background pixels: %d%s", nonbg, NL.c_str());
    for (int py2 = 220; py2 <= 290; py2 += 17) {
        for (int px2 = 40; px2 <= 260; px2 += 55) {
            Vector3 c = fb.pixel(px2, py2);
            std::printf("(%d,%d d=%.3f c=%.2f/%.2f/%.2f) ", px2, py2,
                        fb.depth_at(px2, py2), c.x, c.y, c.z);
        }
        std::printf("%s", NL.c_str());
    }
    FrameBuffer fb800(800, 800);
    // ---- ⑧ 地面面片逐阶段单测：视锥裁剪 → 投影 → W-A ----
    {
        PointLight top;  top.position = {0, 1.8f, 4.5f};   // 复用相机位置当"光源"方便对照
        Mat4 view = Mat4::look_at(top.position, Vector3(0, 0, 0), Vector3(0, 1, 0));
        Mat4 proj = Mat4::perspective(60.0f, 1.0f, 0.1f, 100.0f);
        Mat4 vp = proj * view;
        std::vector<Vec4> quad;
        for (int ix : {16, 17, 18, 19}) quad.push_back(vp.transform_point_clip(mesh_vertex(ix)));
        std::printf("clip-space w: %.3f %.3f %.3f %.3f%s",
                    quad[0].w, quad[1].w, quad[2].w, quad[3].w, NL.c_str());
        std::vector<Vec4> clipped = clip_polygon_frustum(quad, 0.1f);
        std::printf("after frustum clip: %d verts%s", int(clipped.size()), NL.c_str());
        std::vector<Vector3> poly;
        for (auto& c : clipped) {
            Vector3 s = project_to_screen(c, fb800);
            poly.push_back(s);
            std::printf("  (%.0f, %.0f, d=%.3f)%s", s.x, s.y, s.z, NL.c_str());
        }
        Rect win2{0, 0, 800, 800};   // 800² 全屏窗口（与 debug_stairs 一致）
        auto loops = weiler_atherton(std::move(poly), win2);
        std::printf("W-A output loops: %d%s", int(loops.size()), NL.c_str());
        for (auto& lp : loops) {
            std::printf("  loop %d verts:", int(lp.size()));
            for (auto& p : lp) std::printf(" (%.0f,%.0f)", p.x, p.y);
            std::printf("%s", NL.c_str());
        }
    }
    // ---- ⑦ 深度图可视化：导出 512² 深度图为 BMP（灰度，空洞=白）----
    {
        std::ofstream f("build/shadowmap.bmp", std::ios::binary);
        int row = (512 * 3 + 3) & ~3;
        unsigned char hdr[54] = {'B', 'M', 0};
        auto w32 = [&](int o, unsigned int v) { hdr[o] = v & 255;
            hdr[o+1] = (v >> 8) & 255; hdr[o+2] = (v >> 16) & 255; hdr[o+3] = (v >> 24) & 255; };
        w32(2, 54 + row * 512); w32(10, 54); w32(14, 40); w32(18, 512); w32(22, 512);
        w32(26, 1); w32(28, 24); w32(34, row * 512);
        f.write(reinterpret_cast<char*>(hdr), 54);
        std::vector<char> rowbuf(row, 0);
        for (int y = 511; y >= 0; --y) {
            for (int x = 0; x < 512; ++x) {
                float d = smap.depth_at(x, y);
                unsigned char g = (d > 1.0f) ? 255 : (unsigned char)(255 - (1.0f - d) * 255.0f * 40.0f);
                rowbuf[x*3] = rowbuf[x*3+1] = rowbuf[x*3+2] = char(g);
            }
            f.write(rowbuf.data(), row);
        }
        std::printf("shadow map dumped\n");
    }
    return 0;
}
