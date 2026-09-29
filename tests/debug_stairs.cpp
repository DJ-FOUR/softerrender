// 离屏渲染台阶，导出 BMP 供人工检查消隐问题。
// 编译运行：
//   D:/mingw64/bin/g++.exe -std=c++17 -Isrc tests/debug_stairs.cpp -o build/debug_stairs.exe
//   ./build/debug_stairs.exe [yaw] [pitch]
#include "soft_renderer.h"
#include <cstdio>
#include <fstream>
#include <fstream>
#include <cstdlib>

static void save_bmp(const FrameBuffer& fb, const char* path,
                     int cx0, int cy0, int cw, int ch) {
    int w = cw, h = ch;
    int row = (w * 3 + 3) & ~3;
    int data_size = row * h, file_size = 54 + data_size;
    unsigned char hdr[54] = {'B', 'M', 0};
    auto w32 = [&](int o, unsigned int v) {
        hdr[o] = v & 255; hdr[o + 1] = (v >> 8) & 255;
        hdr[o + 2] = (v >> 16) & 255; hdr[o + 3] = (v >> 24) & 255;
    };
    w32(2, file_size); w32(10, 54); w32(14, 40); w32(18, w); w32(22, h);
    w32(26, 1); w32(28, 24); w32(34, data_size);
    std::ofstream f(path, std::ios::binary);
    f.write(reinterpret_cast<char*>(hdr), 54);
    std::vector<char> rowbuf(row, 0);
    for (int y = ch - 1; y >= 0; --y) {          // BMP 行序自下而上
        for (int x = 0; x < cw; ++x) {
            Vector3 c = fb.pixel(cx0 + x, cy0 + y);
            rowbuf[x * 3 + 0] = char(c.z * 255.0f);
            rowbuf[x * 3 + 1] = char(c.y * 255.0f);
            rowbuf[x * 3 + 2] = char(c.x * 255.0f);
        }
        f.write(rowbuf.data(), row);
    }
}

// 判断颜色是否接近橙色前截面填充 (0.90, 0.55, 0.15)
static bool is_orange(const Vector3& c) {
    return std::fabs(c.x - 0.90f) < 0.05f && std::fabs(c.y - 0.55f) < 0.05f &&
           std::fabs(c.z - 0.15f) < 0.05f;
}

// 判断颜色是否接近踏步2 顶面填充 (0.20, 0.60, 0.30)
static bool is_tread2(const Vector3& c) {
    return std::fabs(c.x - 0.20f) < 0.05f && std::fabs(c.y - 0.60f) < 0.05f &&
           std::fabs(c.z - 0.30f) < 0.05f;
}

static std::string NL() { return std::string(1, char(92)) + "n"; }

int main(int argc, char** argv) {
    float yaw = (argc > 1) ? float(std::atof(argv[1])) : 0.0f;
    float pitch = (argc > 2) ? float(std::atof(argv[2])) : 0.0f;
    float lx = (argc > 3) ? float(std::atof(argv[3])) : 1.2f;
    float ly = (argc > 4) ? float(std::atof(argv[4])) : 2.0f;
    float lz = (argc > 5) ? float(std::atof(argv[5])) : 1.8f;

    FrameBuffer fb(800, 800);
    ClipWindow win(1.0f);   // 全屏窗口，便于观察阴影
    SolidMesh stairs;

    PointLight light;  light.position = {lx, ly, lz};
    Camera cam;
    stairs.draw(fb, win, light, cam);  // 相机可转
    {
        // 阴影 on/off 对比统计（阴影只应变暗）
        std::vector<float> on_px(800 * 800);
        for (int y = 0; y < 800; ++y)
            for (int x = 0; x < 800; ++x) {
                Vector3 c = fb.pixel(x, y);
                on_px[y * 800 + x] = (c.x + c.y + c.z) / 3.0f;
            }
        stairs.enable_shadows(false);
        stairs.draw(fb, win, light, cam);  // 相机可转
        stairs.enable_shadows(true);
        int diff = 0, viol = 0, darker = 0;
        for (int y = 0; y < 800; ++y)
            for (int x = 0; x < 800; ++x) {
                Vector3 c = fb.pixel(x, y);
                float b = (c.x + c.y + c.z) / 3.0f;
                if (std::fabs(b - on_px[y * 800 + x]) > 1e-4f) ++diff;
                if (b > on_px[y * 800 + x] + 1e-4f) ++viol;
                if (on_px[y * 800 + x] - b > 0.02f) ++darker;
            }
        std::printf("shadow stats: diff=%d darker=%d viol=%d%s",
                    diff, darker, viol, NL().c_str());
        stairs.draw(fb, win, light, cam);  // 相机可转   // 恢复阴影开画面

        // 深度图可视化：自适应 fov 下圆盘应实心无空洞
        const FrameBuffer& sm = stairs.shadow_map();
        int filled = 0, minx = 9999, maxx = -1, miny = 9999, maxy = -1;
        for (int y = 0; y < 512; ++y)
            for (int x = 0; x < 512; ++x)
                if (sm.depth_at(x, y) < 1.0e6f) {
                    ++filled;
                    minx = std::min(minx, x); maxx = std::max(maxx, x);
                    miny = std::min(miny, y); maxy = std::max(maxy, y);
                }
        std::printf("internal shadow map: filled=%d bbox=(%d,%d)-(%d,%d)%s",
                    filled, minx, miny, maxx, maxy, NL().c_str());
        std::ofstream f("build/internal_map.bmp", std::ios::binary);
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
                float d = sm.depth_at(x, y);
                unsigned char g = (d > 1.0e5f) ? 255
                    : (unsigned char)(128 - (0.99f - d) * 400.0f);
                rowbuf[x*3] = rowbuf[x*3+1] = rowbuf[x*3+2] = char(g);
            }
            f.write(rowbuf.data(), row);
        }

        // 探针：暗区像素的完整重投影链路（与着色器同款数学）
        Mat4 lview2 = Mat4::look_at(light.position, Vector3(0, 0, 0), Vector3(0, 1, 0));
        Mat4 lproj2 = Mat4::perspective(100.0f, 1.0f, 0.1f, 60.0f);
        Mat4 lvp2 = lproj2 * lview2;
        Mat4 view2 = Mat4::look_at(Vector3(0, 1.8f, 4.5f), Vector3(0, 0, 0), Vector3(0, 1, 0));
        Mat4 proj2 = Mat4::perspective(60.0f, 1.0f, 0.1f, 100.0f);
        float kC = proj2.m[10], kE = proj2.m[11];
        Vector3 fwd = (Vector3(0, 0, 0) - Vector3(0, 1.8f, 4.5f)).normalized();
        Vector3 rgt = fwd.cross(Vector3(0, 1, 0)).normalized();
        Vector3 upv = rgt.cross(fwd).normalized();
        int pxs[4] = {400, 300, 500, 400};
        int pys[4] = {600, 500, 700, 450};
        for (int q = 0; q < 4; ++q) {
            int px2 = pxs[q], py2 = pys[q];
            float dz = fb.depth_at(px2, py2);
            if (dz >= 1.0e6f) {
                std::printf("probe (%d,%d): background%s", px2, py2, NL().c_str());
                continue;
            }
            float nx = px2 / 800.0f * 2.0f - 1.0f;
            float ny = 1.0f - py2 / 800.0f * 2.0f;
            float ww = kE / (2.0f * dz - 1.0f + kC);
            Vector3 wp = Vector3(0, 1.8f, 4.5f) + rgt * (nx * ww / proj2.m[0])
                       + upv * (ny * ww / proj2.m[5]) + fwd * ww;
            Vec4 lc = lvp2.transform_point_clip(wp);
            float inv = 1.0f / lc.w;
            int su = int(((lc.x * inv) + 1.0f) * 0.5f * 512);
            int sv = int((1.0f - (lc.y * inv)) * 0.5f * 512);
            float my = (lc.z * inv + 1.0f) * 0.5f;
            float stored = (su >= 0 && su < 512 && sv >= 0 && sv < 512)
                ? 0.0f /* 填充时无法直接读内部图，仅打印 uv 与 my */ : 0.0f;
            std::printf("probe (%d,%d): z01=%.4f w=%.2f world=(%.2f,%.2f,%.2f) lcw=%.3f uv=(%d,%d) my=%.4f%s",
                        px2, py2, dz, ww, wp.x, wp.y, wp.z, lc.w, su, sv, my, NL().c_str());
        }
    }

    // 扫描：踏步2 的暗绿描边像素，且周围没有踏步2 自己的填充
    // → 描边出现在了别人（橙色面）的地盘上 = 真漏线
    int found = 0;
    for (int y = 2; y < 798 && found < 40; ++y) {
        for (int x = 2; x < 798 && found < 40; ++x) {
            Vector3 c = fb.pixel(x, y);
            // 暗绿描边色 = (0.20,0.60,0.30)×0.4 = (0.08,0.24,0.12)
            if (std::fabs(c.x - 0.08f) > 0.03f || std::fabs(c.y - 0.24f) > 0.03f ||
                std::fabs(c.z - 0.12f) > 0.03f) continue;
            int orange_n = 0, tread_n = 0;
            for (int dy = -2; dy <= 2; ++dy)
                for (int dx = -2; dx <= 2; ++dx) {
                    if (is_orange(fb.pixel(x + dx, y + dy))) ++orange_n;
                    if (is_tread2(fb.pixel(x + dx, y + dy))) ++tread_n;
                }
            if (orange_n >= 12 && tread_n == 0) {
                if (found < 12)
                    std::printf("leak! (%4d,%4d) depth=%.4f orange_n=%d\n",
                                x, y, fb.depth_at(x, y), orange_n);
                ++found;
            }
        }
    }
    std::printf("deep-leak pixels: %d\n", found);

    save_bmp(fb, "build/debug_stairs.bmp", 0, 0, 800, 800);
    std::printf("saved build/debug_stairs.bmp (crop 300,280 260x260) yaw=%g pitch=%g\n", yaw, pitch);

    // ---- 深度诊断：漏线像素 vs 周围橙色填充的深度 ----
    std::printf("\n-- depth around leak (410,330): --\n");
    for (int y = 326; y <= 334; y += 2) {
        std::printf("y=%d: ", y);
        for (int x = 404; x <= 420; x += 4)
            std::printf("(x=%d d=%.4f) ", x, fb.depth_at(x, y));
        std::printf("\n");
    }

    // ---- 几何诊断：用同一套矩阵投影踏步2 的 4 个顶点和 4 条边 ----
    {
        const Vector3 eye(0, 1.8f, 4.5f);
        Mat4 view = Mat4::look_at(eye, Vector3(0, 0, 0), Vector3(0, 1, 0));
        Mat4 proj = Mat4::perspective(60.0f, 1.0f, 0.1f, 100.0f);
        // 与 SolidMesh 内部相同的模型矩阵：rotate_world 依次左乘
        // yaw、pitch，矩阵形式为 R = Rx(pitch) · Ry(yaw)
        Mat4 model = Mat4::rotate_x(pitch) * Mat4::rotate_y(yaw);
        Mat4 mvp = proj * view * model;
        // 踏步2 顶点（与 soft_renderer.h 中 faces[6] = {5,4,12,13} 一致）
        const Vector3 tv[4] = {
            {-0.5f, 0.5f, 1}, {0.5f, 0.5f, 1}, {0.5f, 0.5f, -1}, {-0.5f, 0.5f, -1}};
        std::printf("\n-- tread2 projected vertices: --\n");
        for (int i = 0; i < 4; ++i) {
            Vec4 c = mvp.transform_point_clip(tv[i]);
            Vector3 s = project_to_screen(c, fb);
            std::printf("v%d: screen=(%.1f, %.1f) depth01=%.4f\n", i, s.x, s.y, s.z);
        }
    }
    return 0;
}
