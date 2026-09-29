#pragma once
// ============================================================
// 软件渲染器（纯 CPU）—— 当前阶段：OBJ/MTL 模型导入 + 阴影贴图
// 全部渲染计算在本文件完成，输出一张 RGB 浮点帧缓冲；
// main.cpp 用 OpenGL 的 GL_POINTS 把帧缓冲画到屏幕上（仅此而已）。
//
// 面向对象划分：
//   FrameBuffer —— 帧缓冲：像素的存储与写入（clear / plot / draw_line /
//                  fill_polygon-AET 扫描线填充），含 z-buffer 与识别缓冲；
//   ClipWindow  —— 裁剪窗口：屏幕中央的矩形，可缩放，是 W-A 的裁剪区域；
//   SolidMesh   —— 场景对象：OBJ 导入的模型 + 固定地面（解析失败回退
//                  内置悬浮球），私有状态（平移/四元数姿态/缩放）+
//                  公开操作接口（move / rotate_world / rotate_local /
//                  scale_by），draw() 按面走完整管线。
//
// SolidMesh::draw 的管线阶段（一帧的执行顺序）：
//   0. 阴影贴图      Pass 1：从光源视角把全部面渲染进 512² 深度图
//                    （复用 AET 填充，只写深度），供 Pass 2 逐像素查询
//   1. 组合模型矩阵  平移 · 四元数旋转 · 缩放（由键盘状态驱动）
//   2. 顶点变换      model → view → projection → 裁剪空间
//   3. 背面剔除      面法线 · 视线 ≤ 0 的面直接跳过（世界空间，B 键开关）
//   4. 着色          Blinn-Phong（逐像素）：OBJ 的 vn 法线穿过裁剪/光栅化
//                    管线逐像素插值（vn 平→平面着色，vn 滑→平滑着色），
//                    屏幕坐标+NDC 深度重建片元世界坐标，光源空间采样
//                    阴影图（阴影中只剩环境光，C 键开关），再算
//                    (N·L) 漫反射 + (N·H)^s 高光；光源键盘可移动
//   5. 近平面裁剪    每个面在 3D 裁剪空间对 w = W_NEAR 半空间裁剪
//                    （单平面 Sutherland–Hodgman，防止相机穿过时出错）
//   6. 齐次除法      裁剪空间 → NDC
//   7. 视口变换      NDC → 屏幕坐标（y 翻转）
//   8. 窗口裁剪      Weiler–Atherton：把面多边形裁剪进矩形窗口
//                    （多边形对多边形，在两个边界环之间游走）
//   9. 区域填充+消隐 AET 扫描线填充，逐像素 z-buffer 深度测试（H 键开关）。
//                    深度用 NDC z（近 0 远 1）——它沿屏幕直线是线性的，
//                    所以行间 d += d_dy、span 内逐像素线性插值都是精确值；
//                    更近的片元才写入颜色，与面的绘制顺序无关。
//  10. DDA 光栅化    描出多边形轮廓——描边带面的 id，只画在该面的填充
//                    赢下的像素上（识别缓冲门槛），被遮挡的描边自动消失
//  11. 上屏          main.cpp 用 GL_POINTS 画出帧缓冲
//
// 说明：三角形光栅化 + z-buffer + Blinn-Phong 已在上一阶段实现过，
// 归档于 rasterizer_backup.h，后续按学习路线恢复。
// ============================================================
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include "linalg.h"
#include "objloader.h"   // OBJ/MTL 导入：SolidMesh::build_scene 启动时调用
#include "texture.h"     // 纹理加载与采样（map_Kd）

// 带属性的屏幕多边形顶点：光栅化（FrameBuffer）与裁剪（Weiler–Atherton）共用。
//   pos   = (像素x, 像素y, NDC深度)
//   nrm   = 着色法线（世界空间，未归一化）
//   uv    = (u/w, v/w) —— 除过 w 的 UV。透视校正插值的关键：u 本身沿屏幕
//           直线不线性，但 u/w 与 1/w 都是屏幕仿射量，线性插值它们、
//           像素处再相除（u = (u/w)/(1/w)）即得透视正确的 UV——这正是
//           旧项目"贴图游动"bug 的解法。
//   inv_w = 1/w
struct PolyVert {
    Vector3 pos;
    Vector3 nrm;
    Vec2 uv;
    float inv_w = 1.0f;
};

// 帧缓冲：一维数组（idx = y*width + x），缓存友好。
// 线框阶段无遮挡需求，暂不含深度通道；恢复三角形光栅化时再加回。
class FrameBuffer {
public:
    FrameBuffer(int w, int h)
        : width_(w), height_(h),
          r_(size_t(w) * h, 1.0f), g_(size_t(w) * h, 1.0f), b_(size_t(w) * h, 1.0f),
          depth_(size_t(w) * h, kFarDepth), id_(size_t(w) * h, kNoId) {}

    int width() const { return width_; }
    int height() const { return height_; }

    // 消隐开关（H 键）：关闭后退化为画家算法（后画的盖先画的），
    // 用于对比观察 z-buffer 的作用
    void enable_depth_test(bool on) { depth_test_ = on; }
    bool depth_test_enabled() const { return depth_test_; }

    void clear() {
        std::fill(r_.begin(), r_.end(), 1.0f);      // 背景白色
        std::fill(g_.begin(), g_.end(), 1.0f);
        std::fill(b_.begin(), b_.end(), 1.0f);
        std::fill(depth_.begin(), depth_.end(), kFarDepth);   // 深度清成"无限远"
        std::fill(id_.begin(), id_.end(), kNoId);   // 归属清成"无主"
    }

    // 写一个像素（无深度，overlay 用：窗口框等始终可见的元素），越界自动忽略
    void plot(int x, int y, const Vector3& color) {
        if (x < 0 || x >= width_ || y < 0 || y >= height_) return;
        int idx = y * width_ + x;
        r_[idx] = color.x; g_[idx] = color.y; b_[idx] = color.z;
    }

    // 带深度的像素写入——z-buffer 消隐的核心判据：
    // 深度更近（更小）的片元才写入颜色、深度和归属 id，更远的被丢弃。
    // 与绘制顺序无关：先画远的再画近的、或反过来，结果一致。
    void plot(int x, int y, const Vector3& color, float depth, int id) {
        if (x < 0 || x >= width_ || y < 0 || y >= height_) return;
        int idx = y * width_ + x;
        if (depth_test_) {
            if (depth >= depth_[idx]) return;       // 被更近的片元遮挡
            depth_[idx] = depth;
        }
        r_[idx] = color.x; g_[idx] = color.y; b_[idx] = color.z;
        id_[idx] = id;
    }

    // 读一个像素（上屏时用）
    Vector3 pixel(int x, int y) const {
        int idx = y * width_ + x;
        return {r_[idx], g_[idx], b_[idx]};
    }

    // 读一个像素的深度（调试/回归测试用）
    float depth_at(int x, int y) const { return depth_[y * width_ + x]; }

    // 画一条线段：DDA 数值微分法。
    // 原理：沿更长的轴均匀步进（每步约 1 像素），另一轴按斜率累加，
    // 每步四舍五入到一个像素。每步恰好推进一个像素，线不会断也不重复涂。
    // 几何前提：透视投影保持直线性，屏幕空间线性插值不会把线画弯。
    //
    // 像素映射必须用 floor（像素 i 覆盖 [i, i+1)，中心 i+0.5），
    // 与 AET 填充的采样约定严格一致。若用四舍五入（中心=整数），
    // 两条约定差半像素：近面填充的覆盖比真实边界缩了半像素，而描边的
    // 像素恰好散落在这条空洞带里——那里深度缓冲还是"无限远"，
    // 深度测试形同虚设，隐藏面的描边会贴着边界漏出来（实测 bug）。
    void draw_line(float x0, float y0, float x1, float y1, const Vector3& color) {
        int steps = int(std::max(std::fabs(x1 - x0), std::fabs(y1 - y0)));
        if (steps < 1) steps = 1;                     // 两端同像素时至少画一点
        float dx = (x1 - x0) / steps;
        float dy = (y1 - y0) / steps;

        float x = x0, y = y0;
        for (int i = 0; i <= steps; ++i) {
            plot(int(std::floor(x)), int(std::floor(y)), color);
            x += dx; y += dy;
        }
    }

// 带归属的描边线段：轮廓属于某个面，只允许画在"该面的填充赢下"的像素上。
// 像素映射用 floor（与填充一致，理由见上）；逐像素过 id 门槛，不写深度。
// 这是真实编辑器"识别缓冲（id buffer）"的思路：描边的可见性不再靠
// 深度比较——共边面上两面深度打平、偏置翻盘之类的歧义从根上消失。
void draw_line(float x0, float y0, float x1, float y1,
               const Vector3& color, int id) {
    int steps = int(std::max(std::fabs(x1 - x0), std::fabs(y1 - y0)));
    if (steps < 1) steps = 1;
    float dx = (x1 - x0) / steps;
    float dy = (y1 - y0) / steps;

    float x = x0, y = y0;
    for (int i = 0; i <= steps; ++i) {
        plot_outline(int(std::floor(x)), int(std::floor(y)), color, id);
        x += dx; y += dy;
    }
}

    // ============================================================
    // 多边形扫描转换：AET 扫描线填充（有序边表法）
    //
    // 输入任意简单多边形（三角形/四边形/裁剪出的多边形都行，无需拆三角形），
    // 把内部逐行涂上 color。id 是该面在识别缓冲里的编号，
    // 随每个像素一并写入，供描边的 id 门槛使用。
    // 核心三步在每条扫描线上重复：
    //   求交（扫描线与各边的 x 交点）→ 排序 → 奇偶配对成 span 填充。
    // 教科书的两个数据结构：
    //   GET（全局边表）—— 按"边变活跃的扫描线号"分桶，一次建好；
    //   AET（活性边表）—— 当前扫描线穿过的边，随扫描线下移增量维护：
    //     排序 → 填 span → 淘汰出界的边（y 越过 ymax）→ 剩余边 x += 1/m。
    //
    // 顶点属性：除位置 (x, y, NDC 深度) 外每个顶点还带着色法线（世界空间），
    // 与深度完全对称地沿边/跨距线性插值，逐像素交给 shader——这就是
    // Gouraud/Phong 平滑着色的插值机制（屏幕空间线性近似，面片小到
    // 一个屏占比以内时与透视校正的差异不可见；透视校正留给贴图阶段）。
    // 平面着色 = 三个角点法线相同的特例，插值后仍是同一根法线。
    //
    // 两个关键约定：
    //   1. 半开区间 [ymin, ymax)：边只在自己"正在穿过"的扫描线上算交点，
    //      顶点处的交点个数由此自动正确（单调顶点算 1 次、极值顶点算 2 次），
    //      无需单独写奇点处理；
    //   2. 采样在像素中心 (i+0.5, k+0.5)：第 k 条扫描线高度 y = k + 0.5，
    //      填充第 k 行；span 内填像素 i 满足 i + 0.5 ∈ [x_left, x_right)。
    // ============================================================

    // shader 在每个像素中心被调用：像素坐标、插值 NDC 深度、插值着色法线
    //（未归一化，shader 自行 normalize）、透视校正后的 UV（可能超出 [0,1]，
    // 采样器自行回绕），返回该像素颜色。
    using PixelShader = std::function<Vector3(float px, float py, float depth,
                                              const Vector3& N, float u, float v)>;

    // 平色填充：常数颜色，法线与 UV 参数被忽略
    void fill_polygon(const std::vector<Vector3>& poly, const Vector3& color, int id) {
        fill_polygon(poly, id, [color](float, float, float, const Vector3&, float, float) {
            return color;
        });
    }

    // 兼容重载：纯位置多边形（无属性），法线填零。测试与无属性绘制用。
    void fill_polygon(const std::vector<Vector3>& poly, int id, const PixelShader& shader) {
        std::vector<PolyVert> verts;
        verts.reserve(poly.size());
        for (const Vector3& p : poly) verts.push_back({p, Vector3(0, 0, 0)});
        fill_polygon(verts, id, shader);
    }

    void fill_polygon(const std::vector<PolyVert>& poly, int id, const PixelShader& shader) {
        const int n = int(poly.size());
        if (n < 3) return;

        // 活性边表的最小单元：一条"正在被扫描线穿过"的边。
        // 深度与法线成员都和 x 完全对称：NDC 深度沿屏幕直线线性、法线取
        // 屏幕线性近似，所以行间用增量推进、span 内用线性插值。
        // UV 走透视校正通道：插值的是 (u/w, v/w) 与 1/w——它们才是屏幕
        // 仿射量，像素处相除还原 u/v（详见 PolyVert 注释）。
        struct Edge {
            float x;        // 与当前扫描线的交点 x（每行增量推进）
            float dx_dy;    // 斜率倒数 1/m = dx/dy：扫描线下移一行，x 的增量
            float ymax;     // 边的 y 上限（半开区间，不含；ymin 由 GET 桶号隐含）
            float d;        // 与当前扫描线的交点深度（每行增量推进）
            float d_dy;     // 扫描线下移一行，深度的增量
            Vector3 n;      // 与当前扫描线交点的着色法线（每行增量推进）
            Vector3 dn_dy;  // 扫描线下移一行，法线的增量
            Vec2 uv;        // 交点处的 (u/w, v/w)
            Vec2 duv_dy;    // 扫描线下移一行，(u/w, v/w) 的增量
            float iw;       // 交点处的 1/w
            float diw_dy;   // 扫描线下移一行，1/w 的增量
        };

        // ---- ① 建全局边表 GET：按"进入 AET 的扫描线号"分桶 ----
        // 边在 k + 0.5 ≥ ymin 的第一条扫描线活跃 → 桶号 ceil(ymin − 0.5)；
        // 水平边不与扫描线"穿越"相交（交点由相邻边给出），直接跳过。
        std::vector<std::vector<Edge>> get(height_ + 2);
        int k_min = height_, k_max = -1;                 // 本多边形占据的扫描线范围
        for (int i = 0; i < n; ++i) {
            const PolyVert& a = poly[i];
            const PolyVert& b = poly[(i + 1) % n];
            if (a.pos.y == b.pos.y) continue;            // 水平边
            const bool a_lower = (a.pos.y < b.pos.y);
            const float ymin   = a_lower ? a.pos.y : b.pos.y;
            const float ymax   = a_lower ? b.pos.y : a.pos.y;
            const float x_ymin = a_lower ? a.pos.x : b.pos.x;    // 较低端的 x（交点起点）
            const float dx_dy  = (b.pos.x - a.pos.x) / (b.pos.y - a.pos.y);
            const float d_ymin = a_lower ? a.pos.z : b.pos.z;    // 较低端点的深度
            const float d_dy   = (b.pos.z - a.pos.z) / (b.pos.y - a.pos.y);
            const Vector3 n_ymin = a_lower ? a.nrm : b.nrm;      // 较低端点的法线
            const Vector3 dn_dy  = (b.nrm - a.nrm) / (b.pos.y - a.pos.y);
            const Vec2 uv_ymin   = a_lower ? a.uv : b.uv;        // 较低端点的 (u/w, v/w)
            const Vec2 duv_dy    = (b.uv - a.uv) / (b.pos.y - a.pos.y);
            const float iw_ymin  = a_lower ? a.inv_w : b.inv_w;  // 较低端点的 1/w
            const float diw_dy   = (b.inv_w - a.inv_w) / (b.pos.y - a.pos.y);
            int k = int(std::ceil(ymin - 0.5f));
            if (k < 0) k = 0;                            // 面已被窗口裁剪，防御性钳制
            // 关键守卫：若连第一条候选扫描线都越过了 ymax，说明这条边在
            // y 向短于扫描线间距（近水平的薄边），不与任何像素中心相交。
            // 不跳过的话，交点 x 会沿边方向"外推"到 ymax 之外很远，
            // 产生横贯画面的错误 span（近水平薄面在掠射角下的长横线 bug）。
            if (float(k) + 0.5f >= ymax) continue;
            if (k >= height_) continue;
            // 交点属性直接算到本扫描线高度（沿线段线性推进，不外推越界）
            const float t = float(k) + 0.5f - ymin;
            Edge e{x_ymin + t * dx_dy, dx_dy, ymax,
                   d_ymin + t * d_dy, d_dy,
                   n_ymin + dn_dy * t, dn_dy,
                   uv_ymin + duv_dy * t, duv_dy,
                   iw_ymin + diw_dy * t, diw_dy};
            get[size_t(k)].push_back(e);
            if (k < k_min) k_min = k;
            const int k_end = int(std::ceil(ymax - 0.5f));   // 离开 AET 的扫描线号（不含）
            if (k_end > k_max) k_max = k_end;
        }
        if (k_max < k_min) return;                       // 整个多边形在屏幕外

        // ---- ② 逐扫描线维护 AET 并填充 ----
        std::vector<Edge> aet;
        for (int k = k_min; k < k_max && k < height_; ++k) {
            // (a) 把本行变活跃的边并入 AET（x 已在上面算到本行高度）
            for (const Edge& e : get[size_t(k)]) aet.push_back(e);

            // (b) 按 x 排序
            std::sort(aet.begin(), aet.end(),
                      [](const Edge& p, const Edge& q) { return p.x < q.x; });

            // (c) 奇偶配对填充：第 0-1 交点之间、第 2-3 交点之间……
            //     交点数恒为偶数（半开区间约定保证）；浮点巧合出现奇数时
            //     丢弃最后一个，保证不越界。
            for (size_t i = 0; i + 1 < aet.size(); i += 2)
                fill_span(k, aet[i].x, aet[i + 1].x, aet[i].d, aet[i + 1].d,
                          aet[i].n, aet[i + 1].n,
                          aet[i].uv, aet[i + 1].uv, aet[i].iw, aet[i + 1].iw,
                          id, shader);

            // (d) 淘汰：下一条扫描线 (k+1)+0.5 越过边 ymax 的移出 AET
            const float y_next = float(k) + 1.5f;
            aet.erase(std::remove_if(aet.begin(), aet.end(),
                                     [&](const Edge& e) { return y_next >= e.ymax; }),
                      aet.end());

            // (e) 增量：活下来的边沿自身推进一行（全部属性都不重新求交）
            for (Edge& e : aet) {
                e.x += e.dx_dy; e.d += e.d_dy;
                e.n += e.dn_dy; e.uv += e.duv_dy; e.iw += e.diw_dy;
            }
        }
    }

private:
    // 描边像素：只落在"自己面的填充赢下"的像素（id 匹配），或尚无主
    // 的背景像素（kNoId——多边形最外圈边界落在最后一个填充中心之外，
    // 那里几何上就是这条边的位置）。不写深度、不写归属，纯颜色覆盖。
    void plot_outline(int x, int y, const Vector3& color, int id) {
        if (x < 0 || x >= width_ || y < 0 || y >= height_) return;
        int idx = y * width_ + x;
        if (id_[idx] != id && id_[idx] != kNoId) return;   // 别人的地盘：被遮挡
        r_[idx] = color.x; g_[idx] = color.y; b_[idx] = color.z;
    }

    // 填一个水平区间 [x0, x1)：像素中心 i+0.5 落在区间内的像素。
    // 端点各带 NDC 深度、着色法线与 (u/w, v/w) + 1/w，逐像素线性插值；
    // UV 在像素处相除还原（透视校正），交给 shader。
    void fill_span(int y, float x0, float x1, float d0, float d1,
                   const Vector3& n0, const Vector3& n1,
                   const Vec2& uv0, const Vec2& uv1, float iw0, float iw1,
                   int id, const PixelShader& shader) {
        int i0 = int(std::ceil(x0 - 0.5f));
        int i1 = int(std::ceil(x1 - 0.5f)) - 1;
        const int xa = std::max(i0, 0);
        const int xb = std::min(i1, width_ - 1);
        if (xa > xb) return;
        const float dd = (x1 - x0 > 1e-6f) ? (d1 - d0) / (x1 - x0) : 0.0f;
        const Vector3 dn = (x1 - x0 > 1e-6f) ? (n1 - n0) / (x1 - x0) : Vector3(0, 0, 0);
        const Vec2 duv = (x1 - x0 > 1e-6f) ? (uv1 - uv0) / (x1 - x0) : Vec2();
        const float diw = (x1 - x0 > 1e-6f) ? (iw1 - iw0) / (x1 - x0) : 0.0f;
        const float s = float(xa) + 0.5f - x0;         // 首像素到区间左端的偏移
        float d = d0 + dd * s;
        Vector3 n = n0 + dn * s;
        Vec2 uv = uv0 + duv * s;
        float iw = iw0 + diw * s;
        for (int x = xa; x <= xb; ++x) {
            const float inv = (iw > 1e-8f) ? 1.0f / iw : 0.0f;   // 透视校正还原 UV
            plot(x, y, shader(x + 0.5f, float(y) + 0.5f, d, n, uv.x * inv, uv.y * inv),
                 d, id);
            d += dd; n += dn; uv += duv; iw += diw;
        }
    }

    // 深度清空值：合法深度是 NDC [0,1]（近 0 远 1），
    // 初始化成远大于 1 的数 = "无限远"，保证第一个片元总能写入
    static constexpr float kFarDepth = 1.0e6f;
    static constexpr int kNoId = -1;     // id 通道的"无主"标记

    int width_, height_;
    std::vector<float> r_, g_, b_;
    std::vector<float> depth_;           // z-buffer：每像素存 NDC 深度
    std::vector<int> id_;                // 识别缓冲：每像素存"胜出面"编号
    bool depth_test_ = true;             // 消隐开关
};

// ============================================================
// 裁剪窗口：屏幕中央的可缩放矩形
// ============================================================
struct Rect {
    float xmin = 0, ymin = 0, xmax = 0, ymax = 0;
};

class ClipWindow {
public:
    ClipWindow() = default;
    explicit ClipWindow(float scale) : scale_(scale) {}

    float scale() const { return scale_; }

    // 调整窗口占屏比例，钳制范围防止窗口缩没或溢出屏幕
    void scale_by(float factor) {
        scale_ = std::min(std::max(scale_ * factor, kMinScale), kMaxScale);
    }

    // 窗口矩形（像素坐标，y 向下与帧缓冲一致）
    Rect rect(const FrameBuffer& fb) const {
        float half_w = fb.width() * 0.5f * scale_;
        float half_h = fb.height() * 0.5f * scale_;
        float cx = fb.width() * 0.5f, cy = fb.height() * 0.5f;
        return {cx - half_w, cy - half_h, cx + half_w, cy + half_h};
    }

    // 画窗口框（淡红色，与绿色线框立方体区分）
    void draw(FrameBuffer& fb) const {
        Rect r = rect(fb);
        const Vector3 color(0.78f, 0.35f, 0.35f);
        fb.draw_line(r.xmin, r.ymin, r.xmax, r.ymin, color);
        fb.draw_line(r.xmax, r.ymin, r.xmax, r.ymax, color);
        fb.draw_line(r.xmax, r.ymax, r.xmin, r.ymax, color);
        fb.draw_line(r.xmin, r.ymax, r.xmin, r.ymin, color);
    }

private:
    static constexpr float kMinScale = 0.15f;
    static constexpr float kMaxScale = 1.0f;
    float scale_ = 0.7f;    // 窗口边长占屏幕边长的比例
};

// ============================================================
// 二维几何辅助（Weiler–Atherton 用）
// ============================================================

// 多边形有向面积（鞋带公式）：符号代表顶点环的方向
inline float signed_area(const std::vector<Vector3>& poly) {
    float s = 0;
    for (size_t i = 0; i < poly.size(); ++i) {
        const Vector3& a = poly[i];
        const Vector3& b = poly[(i + 1) % poly.size()];
        s += a.x * b.y - b.x * a.y;
    }
    return s * 0.5f;
}

inline bool point_in_rect(const Vector3& p, const Rect& r) {
    const float eps = 1e-3f;
    return p.x >= r.xmin - eps && p.x <= r.xmax + eps &&
           p.y >= r.ymin - eps && p.y <= r.ymax + eps;
}

// 点在凸多边形内：对每条边做叉积，符号全部一致即在内部（凸性保证）
inline bool point_in_convex_poly(const Vector3& p, const std::vector<Vector3>& poly) {
    float sign = 0;
    for (size_t i = 0; i < poly.size(); ++i) {
        const Vector3& a = poly[i];
        const Vector3& b = poly[(i + 1) % poly.size()];
        float cr = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
        if (std::fabs(cr) < 1e-9f) continue;
        float s = (cr > 0.0f) ? 1.0f : -1.0f;
        if (sign == 0) sign = s;
        else if (s != sign) return false;
    }
    return true;
}

inline float dist2d(const Vector3& p, const Vector3& q) {
    const float dx = p.x - q.x, dy = p.y - q.y;
    return std::sqrt(dx * dx + dy * dy);
}

inline bool same_point(const Vector3& p, const Vector3& q) {
    return std::fabs(p.x - q.x) < 1e-4f && std::fabs(p.y - q.y) < 1e-4f;
}

// 线段 ab 与线段 cd 的交点：t 沿 ab，u 沿 cd，均要求落在 [0,1]。
// 解 a + t·r = c + u·s（r、s 为两边方向向量），叉积求参，平行时无解。
inline bool seg_seg_intersect(const Vector3& a, const Vector3& b,
                              const Vector3& c, const Vector3& d,
                              float& t_out, float& u_out) {
    float rx = b.x - a.x, ry = b.y - a.y;
    float sx = d.x - c.x, sy = d.y - c.y;
    float denom = rx * sy - ry * sx;
    if (std::fabs(denom) < 1e-9f) return false;      // 平行：无交点
    float qpx = c.x - a.x, qpy = c.y - a.y;
    float t = (qpx * sy - qpy * sx) / denom;
    float u = (qpx * ry - qpy * rx) / denom;
    if (t < 0.0f || t > 1.0f || u < 0.0f || u > 1.0f) return false;
    t_out = t; u_out = u;
    return true;
}

inline std::vector<Vector3> rect_corners(const Rect& r) {
    return {{r.xmin, r.ymin, 0}, {r.xmax, r.ymin, 0},
            {r.xmax, r.ymax, 0}, {r.xmin, r.ymax, 0}};
}

// ============================================================
// Weiler–Atherton 多边形裁剪
//
// 把 subject 多边形裁剪到矩形窗口 rect 内，输出 0 个或多个结果多边形。
// 与 Sutherland–Hodgman 的"逐边裁"不同，W-A 直接在两个多边形的
// 边界环之间游走，天然支持凹多边形、结果自动分成若干独立环。
// （本场景 subject 恒为凸——透视投影保持凸性——输出只有一个环，
//   但算法按一般情形实现，将来换凹测试体也能直接用。）
//
// 遍历规则（关键约定：先把两个环归一成同一方向）：
//   - 沿 subject 前进由外入内的交点标记为 ENTERING；
//   - 从任一未访问的 ENTERING 交点出发，沿 subject 环前进，
//     遇到 EXITING 交点就跳到 clip 环上继续前进，
//     再遇到 ENTERING 交点跳回 subject 环……直到回到出发点，
//     沿途经过的顶点按顺序收集，就是一个输出环；
//   - 还有未访问的 ENTERING 交点就再来一轮（凹情形会出多个环）。
// ============================================================

// 环上的节点：原顶点或交点（位置 + 随行的着色法线）
struct WANode {
    PolyVert v;
    bool is_intersection = false;
    int inter_id = -1;          // is_intersection 时指向交点编号
};

// 交点记录：同时登记它在两个环上的位置
struct WAIntersection {
    PolyVert v;
    bool entering = false;      // 沿 subject 前进方向由外入内？
    int subject_index = -1;     // 在 subject 环上的节点下标
    int clip_index = -1;        // 在 clip 环上的节点下标
    bool visited = false;
};

// Weiler–Atherton 多边形裁剪（带着色法线版）：把 subject 裁剪到矩形窗口
// rect 内，输出 0 个或多个结果环。顶点法线作为"骑在顶点上的属性"处理：
// 交点处的法线沿 subject 边按 t 线性插值（与深度的插值方式一致）；
// 窗口角点是虚构顶点，法线取环上相邻两交点法线的平均（仅影响贴着窗口
// 边界的窄条区域的着色，属可接受的近似）。
inline std::vector<std::vector<PolyVert>> weiler_atherton(const std::vector<PolyVert>& subject_in,
                                                          const Rect& rect) {
    std::vector<std::vector<PolyVert>> results;
    if (subject_in.size() < 3) return results;

    std::vector<PolyVert> subject = subject_in;

    // ---- 0. 方向归一：subject 与矩形环转成同一方向 ----
    // 两个环同向时，"沿环前进"在几何上才有一致的内外语义。
    const auto area_of = [](const std::vector<PolyVert>& p) {
        float s = 0;
        for (size_t i = 0; i < p.size(); ++i) {
            const Vector3& a = p[i].pos;
            const Vector3& b = p[(i + 1) % p.size()].pos;
            s += a.x * b.y - b.x * a.y;
        }
        return s * 0.5f;
    };
    std::vector<PolyVert> clip_poly;
    {
        const std::vector<Vector3> corners = rect_corners(rect);
        for (const Vector3& c : corners) clip_poly.push_back({c, Vector3(0, 0, 0)});
    }
    if (area_of(subject) * area_of(clip_poly) < 0.0f)
        std::reverse(subject.begin(), subject.end());

    // ---- 0.5 平面深度函数：NDC 深度沿屏幕平面仿射（A*x + B*y + D）----
    // 用主体多边形的前三个顶点解出系数。输出环里来自窗口边框的角点是
    // 虚构点（没有真实深度），之前无论填 0 还是插值都会毒化整面的深度
    // 插值（实测：跨窗口的地面整片深度趋零、阴影判断全错）。
    // 所有面都是平面，因此用这个仿射函数重算输出顶点深度是精确的。
    float plane_A = 0.0f, plane_B = 0.0f, plane_D = 0.0f;
    {
        const Vector3& p0 = subject[0].pos;
        const Vector3& p1 = subject[1 % subject.size()].pos;
        const Vector3& p2 = subject[2 % subject.size()].pos;
        const float det = (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y);
        if (std::fabs(det) > 1e-9f) {
            plane_A = ((p1.z - p0.z) * (p2.y - p0.y) - (p2.z - p0.z) * (p1.y - p0.y)) / det;
            plane_B = ((p2.z - p0.z) * (p1.x - p0.x) - (p1.z - p0.z) * (p2.x - p0.x)) / det;
            plane_D = p0.z - plane_A * p0.x - plane_B * p0.y;
        } else {
            plane_A = std::numeric_limits<float>::quiet_NaN();   // 侧棱面：标记失效
        }
    }

    // ---- 1. 求 subject 各边与矩形 4 条边的交点 ----
    // {t 沿 subject 边, u 沿 clip 边, 位置+属性, subject 边号, clip 边号}
    struct Hit { float t, u; PolyVert v; int subj_edge, clip_edge; };
    std::vector<Hit> hits;
    const int ns = int(subject.size());
    for (int i = 0; i < ns; ++i) {
        const PolyVert& a = subject[i];
        const PolyVert& b = subject[(i + 1) % ns];
        for (int k = 0; k < 4; ++k) {
            const Vector3& c = clip_poly[k].pos;
            const Vector3& d = clip_poly[(k + 1) % 4].pos;
            float t, u;
            if (seg_seg_intersect(a.pos, b.pos, c, d, t, u))
                hits.push_back({t, u,
                                {a.pos + (b.pos - a.pos) * t,    // 深度沿线段同步插值
                                 a.nrm + (b.nrm - a.nrm) * t,    // 法线沿边同步插值
                                 a.uv + (b.uv - a.uv) * t,       // (u/w, v/w) 同步插值
                                 a.inv_w + (b.inv_w - a.inv_w) * t},
                                i, k});
        }
    }

    // 同一物理交点可能被相邻的两条边各报一次（端点恰在边界上），去重。
    // 注：切点恰好压在顶点上的病态情形按 eps 近似处理，作业精度足够。
    std::vector<Hit> uniq;
    for (const Hit& h : hits) {
        bool dup = false;
        for (const Hit& g : uniq)
            if (same_point(h.v.pos, g.v.pos)) { dup = true; break; }
        if (!dup) uniq.push_back(h);
    }

    // 无交点的三种特判：subject 全在窗口内 / 全在窗口外 / 包住整个窗口
    if (uniq.empty()) {
        if (point_in_rect(subject[0].pos, rect)) {
            results.push_back(subject);
        } else {
            std::vector<Vector3> subj_pos;
            subj_pos.reserve(subject.size());
            for (const PolyVert& v : subject) subj_pos.push_back(v.pos);
            if (point_in_convex_poly(clip_poly[0].pos, subj_pos)) {
                // subject 包住窗口：输出窗口环，角点法线/UV/深度都用
                // subject[0] 的属性兜底（能走到这里说明面比窗口大得多，
                // 窄条近似足够）。
                std::vector<PolyVert> loop = clip_poly;
                for (auto& p : loop) {
                    if (!std::isnan(plane_A))
                        p.pos.z = plane_A * p.pos.x + plane_B * p.pos.y + plane_D;
                    p.nrm = subject[0].nrm;
                    p.uv = subject[0].uv;
                    p.inv_w = subject[0].inv_w;
                }
                results.push_back(std::move(loop));
            }
        }
        return results;
    }

    // ---- 2. 构造 subject 环（顶点 + 交点按 t 排序插入），标记进/出 ----
    // 内外状态沿环连续传递：每穿过一个交点翻转一次，
    // 翻转后在窗口内 = ENTERING，在窗口外 = EXITING。
    std::vector<WANode> subj_ring;
    std::vector<WAIntersection> inters(uniq.size());
    for (int q = 0; q < int(uniq.size()); ++q) inters[q].v = uniq[q].v;

    bool inside = point_in_rect(subject[0].pos, rect);
    for (int i = 0; i < ns; ++i) {
        subj_ring.push_back({subject[i], false, -1});
        std::vector<int> edge_hits;                       // 该边上的交点编号
        for (int q = 0; q < int(uniq.size()); ++q)
            if (uniq[q].subj_edge == i) edge_hits.push_back(q);
        std::sort(edge_hits.begin(), edge_hits.end(),
                  [&](int p, int r) { return uniq[p].t < uniq[r].t; });
        for (int q : edge_hits) {
            inside = !inside;
            inters[q].entering = inside;
            inters[q].subject_index = int(subj_ring.size());
            subj_ring.push_back({uniq[q].v, true, q});
        }
    }

    // ---- 3. 构造 clip 环（矩形顶点 + 交点按 u 排序插入）----
    // 窗口角点是虚构顶点：法线取环上相邻交点法线的平均（无邻交点时用
    // subject[0] 的法线兜底），深度由函数末尾的平面深度函数统一重校准。
    std::vector<WANode> clip_ring;
    for (int k = 0; k < 4; ++k) {
        clip_ring.push_back({clip_poly[k], false, -1});
        std::vector<int> edge_hits;
        for (int q = 0; q < int(uniq.size()); ++q)
            if (uniq[q].clip_edge == k) edge_hits.push_back(q);
        std::sort(edge_hits.begin(), edge_hits.end(),
                  [&](int p, int r) { return uniq[p].u < uniq[r].u; });
        for (int q : edge_hits) {
            inters[q].clip_index = int(clip_ring.size());
            clip_ring.push_back({uniq[q].v, true, q});
        }
    }
    for (size_t i = 0; i < clip_ring.size(); ++i) {
        if (clip_ring[i].is_intersection) continue;
        // 沿环向两侧各找最近的交点（回绕），取平均作为角点属性
        const PolyVert* prev = nullptr;
        const PolyVert* next = nullptr;
        for (size_t j = i; j > 0 && !prev; --j)
            if (clip_ring[j - 1].is_intersection) prev = &clip_ring[j - 1].v;
        for (size_t j = i + 1; j < clip_ring.size() && !next; ++j)
            if (clip_ring[j].is_intersection) next = &clip_ring[j].v;
        if (!prev)   // 回绕到环尾找"前一个"交点
            for (size_t j = clip_ring.size(); j > i + 1 && !prev; --j)
                if (clip_ring[j - 1].is_intersection) prev = &clip_ring[j - 1].v;
        if (!next)   // 回绕到环头找"后一个"交点
            for (size_t j = 0; j < i && !next; ++j)
                if (clip_ring[j].is_intersection) next = &clip_ring[j].v;
        if (prev && next) {   // 相邻交点属性取平均（法线不归一化，shader 处理）
            clip_ring[i].v.nrm = (prev->nrm + next->nrm) * 0.5f;
            clip_ring[i].v.uv = (prev->uv + next->uv) * 0.5f;
            clip_ring[i].v.inv_w = (prev->inv_w + next->inv_w) * 0.5f;
        } else {
            const PolyVert& src = prev ? *prev : next ? *next : subject[0];
            clip_ring[i].v.nrm = src.nrm;
            clip_ring[i].v.uv = src.uv;
            clip_ring[i].v.inv_w = src.inv_w;
        }
    }

    // ---- 4. 遍历：从每个未访问的 ENTERING 交点拉出一个输出环 ----
    for (int q = 0; q < int(inters.size()); ++q) {
        if (!inters[q].entering || inters[q].visited) continue;

        std::vector<PolyVert> loop;
        int cur = q;
        const int start = q;
        int safety = int(subj_ring.size() + clip_ring.size()) * 2;   // 防死循环保险

        do {
            WAIntersection& it = inters[cur];
            it.visited = true;
            loop.push_back(it.v);

            int j;
            if (it.entering) {
                // 在 ENTERING 点沿 subject 环前进，直到下一个交点
                j = (it.subject_index + 1) % int(subj_ring.size());
                while (!subj_ring[j].is_intersection) {
                    loop.push_back(subj_ring[j].v);
                    j = (j + 1) % int(subj_ring.size());
                }
                cur = subj_ring[j].inter_id;
            } else {
                // 在 EXITING 点跳到 clip 环前进，直到下一个交点。
                // 沿途矩形角点是窗口边界上的虚构点，深度不可靠——
                // 输出顶点的深度统一在函数末尾按"平面深度函数"重校准。
                j = (it.clip_index + 1) % int(clip_ring.size());
                while (!clip_ring[j].is_intersection) {
                    loop.push_back(clip_ring[j].v);
                    j = (j + 1) % int(clip_ring.size());
                }
                cur = clip_ring[j].inter_id;
            }
        } while (cur != start && --safety > 0);

        // 丢弃退化环（切点导致的零面积环）
        if (loop.size() >= 3) {
            float area = 0;
            for (size_t i = 0; i < loop.size(); ++i) {
                const Vector3& a = loop[i].pos;
                const Vector3& b = loop[(i + 1) % loop.size()].pos;
                area += a.x * b.y - b.x * a.y;
            }
            if (std::fabs(area * 0.5f) > 1e-5f) {
                // 深度重校准：窗口角点等虚构顶点的深度按平面深度函数重算
                if (!std::isnan(plane_A))
                    for (auto& p : loop)
                        p.pos.z = plane_A * p.pos.x + plane_B * p.pos.y + plane_D;
                results.push_back(std::move(loop));
            }
        }
    }

    return results;
}

// ============================================================
// 3D 管线辅助
// ============================================================

// 多边形对单个半空间（w >= w_near）的裁剪。
// 这是 Sutherland–Hodgman 的单平面最小特例：逐边检查两端点的内外组合，
// 输出"内侧端点 + 边与平面的交点"。输入输出都是裁剪空间齐次坐标，
// 凸多边形裁剪后仍是凸多边形。
inline std::vector<Vec4> clip_polygon_near(const Vec4* poly, size_t n, float w_near) {
    std::vector<Vec4> out;
    for (size_t i = 0; i < n; ++i) {
        const Vec4& a = poly[i];
        const Vec4& b = poly[(i + 1) % n];
        const bool a_in = a.w >= w_near, b_in = b.w >= w_near;
        if (a_in) out.push_back(a);
        if (a_in != b_in) {                       // 边跨越平面：求交点封口
            float t = (w_near - a.w) / (b.w - a.w);
            out.push_back({a.x + t * (b.x - a.x),
                           a.y + t * (b.y - a.y),
                           a.z + t * (b.z - a.z), w_near});
        }
    }
    return out;
}

// 多边形对单个裁剪空间半空间 a*x+b*y+c*z+d*w+e <= 0 的 Sutherland-Hodgman 裁剪：
// 逐边检查两端点的内外组合，跨越处按 t 线性插值出交点（裁剪空间内线性，精确）。
inline std::vector<Vec4> clip_polygon_halfspace(const std::vector<Vec4>& in,
                                                float a, float b, float c,
                                                float d, float e) {
    std::vector<Vec4> out;
    out.reserve(in.size() + 1);
    for (size_t i = 0; i < in.size(); ++i) {
        const Vec4& p = in[i];
        const Vec4& q = in[(i + 1) % in.size()];
        const float dp = a * p.x + b * p.y + c * p.z + d * p.w + e;
        const float dq = a * q.x + b * q.y + c * q.z + d * q.w + e;
        const bool pin = dp <= 0.0f, qin = dq <= 0.0f;
        if (pin) out.push_back(p);
        if (pin != qin) {
            const float t = dp / (dp - dq);
            out.push_back({p.x + t * (q.x - p.x), p.y + t * (q.y - p.y),
                           p.z + t * (q.z - p.z), p.w + t * (q.w - p.w)});
        }
    }
    return out;
}

// 视锥体裁剪：近平面 + 四个"保护带"半空间（投影后 NDC |x|,|y| <= 2）。
// 保护带的作用：像地面这种延伸到相机身后的面，近平面裁剪留下的交点
// 会投影出 +/-4 万像素的天文坐标——W-A 窗口裁剪和 AET 填充在这种量级上
// 会因 float 精度算错（实测整块地面丢失）。先在 NDC 域收口到 +/-2
// （屏幕坐标 +/-1200 以内），后续算法接触的数字就都正常了。
inline std::vector<Vec4> clip_polygon_frustum(const std::vector<Vec4>& in, float w_near) {
    auto poly = clip_polygon_halfspace(in, 0, 0, 0, -1, w_near);   // w >= w_near
    poly = clip_polygon_halfspace(poly,  1, 0, 0, -2, 0);          // x <=  2w
    poly = clip_polygon_halfspace(poly, -1, 0, 0, -2, 0);          // x >= -2w
    poly = clip_polygon_halfspace(poly,  0, 1, 0, -2, 0);          // y <=  2w
    poly = clip_polygon_halfspace(poly,  0, -1, 0, -2, 0);         // y >= -2w
    return poly;
}

// ============================================================
// 带顶点属性的视锥体裁剪：着色法线"骑"在顶点上随位置走，
// 跨越裁剪平面处与位置按同一个 t 线性插值（裁剪空间内线性，精确）。
// 平滑着色（Phong 法线插值）需要法线穿过整条裁剪管线，由此而来。
// ============================================================
struct ClipVert {
    Vec4 c;        // 裁剪空间坐标
    Vector3 n;     // 着色法线（世界空间，未归一化）
    Vec2 uv;       // 模型 UV（裁剪空间内按 t 线性插值——与 3D 边同一参数，精确）
};

// ClipVert 版单半空间 Sutherland–Hodgman（与 Vec4 版同一数学）
inline std::vector<ClipVert> clip_halfspace_attr(const std::vector<ClipVert>& in,
                                                 float a, float b, float c,
                                                 float d, float e) {
    std::vector<ClipVert> out;
    out.reserve(in.size() + 1);
    for (size_t i = 0; i < in.size(); ++i) {
        const ClipVert& p = in[i];
        const ClipVert& q = in[(i + 1) % in.size()];
        const float dp = a * p.c.x + b * p.c.y + c * p.c.z + d * p.c.w + e;
        const float dq = a * q.c.x + b * q.c.y + c * q.c.z + d * q.c.w + e;
        const bool pin = dp <= 0.0f, qin = dq <= 0.0f;
        if (pin) out.push_back(p);
        if (pin != qin) {
            const float t = dp / (dp - dq);
            out.push_back({{p.c.x + t * (q.c.x - p.c.x),
                            p.c.y + t * (q.c.y - p.c.y),
                            p.c.z + t * (q.c.z - p.c.z),
                            p.c.w + t * (q.c.w - p.c.w)},
                           p.n + (q.n - p.n) * t,
                           p.uv + (q.uv - p.uv) * t});
        }
    }
    return out;
}

// 带法线的视锥体裁剪：近平面 + NDC 保护带（理由见上方 Vec4 版注释）
inline std::vector<ClipVert> clip_polygon_frustum_n(const std::vector<ClipVert>& in,
                                                    float w_near) {
    auto poly = clip_halfspace_attr(in, 0, 0, 0, -1, w_near);   // w >= w_near
    poly = clip_halfspace_attr(poly,  1, 0, 0, -2, 0);          // x <=  2w
    poly = clip_halfspace_attr(poly, -1, 0, 0, -2, 0);          // x >= -2w
    poly = clip_halfspace_attr(poly,  0, 1, 0, -2, 0);          // y <=  2w
    poly = clip_halfspace_attr(poly,  0, -1, 0, -2, 0);         // y >= -2w
    return poly;
}

// 裁剪空间 → 屏幕空间：齐次除法 → NDC → 视口变换（y 翻转）。
// 返回 (像素x, 像素y, NDC深度)：深度映射到 [0,1]（近 0 远 1），存进 z 分量。
// 关键性质：NDC 深度沿屏幕上的直线段是线性的，可放心线性插值——
// 这是 z-buffer 能在扫描线框架里"边插值边测试"的数学前提。
inline Vector3 project_to_screen(const Vec4& c, const FrameBuffer& fb) {
    float inv_w = 1.0f / c.w;
    float x = (c.x * inv_w + 1.0f) * 0.5f * fb.width();
    float y = (1.0f - c.y * inv_w) * 0.5f * fb.height();
    float z = (c.z * inv_w + 1.0f) * 0.5f;
    return {x, y, z};
}

// ============================================================
// 点光源 + Blinn-Phong 光照
// ============================================================

// 点光源：位置由键盘实时移动；三分量强度按颜色通道独立给出
struct PointLight {
    Vector3 position{1.2f, 2.0f, 1.8f};    // 默认：右上前方（在视锥内，标记可见）
    Vector3 ambient{0.18f, 0.18f, 0.18f};  // 环境光：无方向，保证背光面不全黑
    Vector3 diffuse{0.85f, 0.85f, 0.85f};  // 漫反射强度
    Vector3 specular{0.70f, 0.70f, 0.70f}; // 镜面反射强度
    float shininess = 32.0f;               // 高光指数：越大高光越小越锐
};

// Blinn-Phong 着色（flat shading：整个面算一次、一个颜色）。
//   color = albedo⊙ambient + albedo⊙diffuse·(N·L) + spec_color·(N·H)^s
// H = normalize(L + V) 是半角向量——比 Phong 的反射向量法便宜一次
// 向量反射，且高光形状更符合能量分布；ndl ≤ 0 时高光直接置零
// （光从背面来，镜面项没有物理意义）。
// spec_color 与 shininess 单独成参：OBJ 材质（Ks/Ns）从这两个口接入——
// 光源镜面强度 ⊙ 材质 Ks，高光锐度用材质 Ns。
inline Vector3 blinn_phong(const Vector3& pos, const Vector3& normal,
                           const Vector3& albedo, const Vector3& spec_color,
                           float shininess, const PointLight& light,
                           const Vector3& eye) {
    const Vector3 L = (light.position - pos).normalized();
    const Vector3 V = (eye - pos).normalized();
    const Vector3 H = (L + V).normalized();

    const float ndl = std::max(0.0f, normal.dot(L));
    const float nh = (ndl > 0.0f)
        ? std::pow(std::max(0.0f, normal.dot(H)), shininess) : 0.0f;

    Vector3 color = albedo * light.ambient +
                    albedo * light.diffuse * ndl +
                    spec_color * nh;
    return {std::min(1.0f, color.x), std::min(1.0f, color.y), std::min(1.0f, color.z)};
}

// 旧签名：镜面强度与高光指数直接取光源设置（无材质几何/内置回退用）
inline Vector3 blinn_phong(const Vector3& pos, const Vector3& normal,
                           const Vector3& albedo, const PointLight& light,
                           const Vector3& eye) {
    return blinn_phong(pos, normal, albedo, light.specular, light.shininess, light, eye);
}

// ============================================================
// 摄像机：位置 + 偏航/俯仰角（FPS 式第一人称）。
// yaw=0 时朝 -z 看；pitch 钳制 ±89° 防止视线翻转（万向节锁的
// 视觉表现）。W/S 沿视线前后、A/D 沿水平右向左右、QE 摇头、RF 俯仰。
// ============================================================
struct Camera {
    Vector3 position{0, 1.8f, 4.5f};
    float yaw = 0.0f;      // 左右摇头（度）：+ 为向右转
    float pitch = -21.8f;  // 上下点头（度）：+ 为抬头（默认微俯视场景）

    static constexpr float kPitchLimit = 89.0f;
    static constexpr float kDeg2Rad = 3.14159265f / 180.0f;

    Vector3 forward() const {          // 视线方向（含俯仰分量）
        const float cy = std::cos(yaw * kDeg2Rad), sy = std::sin(yaw * kDeg2Rad);
        const float cp = std::cos(pitch * kDeg2Rad), sp = std::sin(pitch * kDeg2Rad);
        return {sy * cp, sp, -cy * cp};
    }
    Vector3 right() const {            // 水平右向（不含俯仰，平移用）
        // right = forward × up（与 look_at 内部的构造一致）。z 分量此前
        // 符号写反：yaw=0 时与正确值恰巧重合难以察觉，偏航一旦不为 0，
        // A/D 平移的方向就左右镜像（实测 bug）。
        const float cy = std::cos(yaw * kDeg2Rad), sy = std::sin(yaw * kDeg2Rad);
        return {cy, 0, sy};
    }
    void turn_yaw(float deg) { yaw += deg; }
    void turn_pitch(float deg) {
        pitch = std::min(std::max(pitch + deg, -kPitchLimit), kPitchLimit);
    }
    void move(const Vector3& delta) { position += delta; }
};

// ============================================================
// 场景网格：可变换模型 + 固定地面。
//
// 几何在启动时由 build_scene 装配：
//   1) 优先从 asset/*.obj 导入（objloader.h / mtlloader.h）。装配规则：
//      - OBJ 里"水平足迹最大且扁平（y 向很薄）"的对象当作固定地面
//        （承接阴影、不做模型变换），其余对象的全部面合成可变换模型；
//      - 模型归一化：等比缩放到最大边长 2（与内置球同尺寸），x/z 居中，
//        底部抬到地面上方 kHoverGap 处（与内置球同款悬浮姿态）；
//      - OBJ 没有可当地面的对象时补一块内置地面；
//      - 打不开/解析不出几何时，整套回退到内置球 + 30×30 地面，
//        保证程序与回归测试在没有 asset/ 的环境下也能跑。
//   2) 材质按面记录（face_mat → materials），Kd/Ks/Ns 接入 Blinn-Phong；
//      map_Kd 只解析路径，纹理采样留待贴图阶段。
//   3) 悬浮测试体（离地间隙）让球形阴影完整可见，是评估阴影质量的
//      经典摆位；OBJ 导入后沿用同一摆位规则。
// ============================================================
class SolidMesh {
public:
    // 世界系平移。模型矩阵最外层是平移，所以永远按屏幕方向移动，
    // 与立方体自身转了多少度无关。
    void move(const Vector3& delta) { position_ += delta; }

    // 绕物体自身轴转 deg 度（局部旋转 / 内旋）：右乘，轴随物体一起走。
    // 这是"拧物体"的直觉——物体侧躺后，"绕自己的横轴翻滚"仍然成立。
    void rotate_local(const Vector3& axis, float deg) {
        orientation_ = orientation_ * Quaternion::from_axis_angle(axis, deg);
        orientation_.normalize();      // 抑制浮点漂移
    }

    // 绕世界轴转 deg 度（全局旋转 / 外旋）：左乘，轴固定在世界坐标系。
    // 这是"按屏幕方向转"的直觉——按偏航键永远是绕竖直轴转。
    void rotate_world(const Vector3& axis, float deg) {
        orientation_ = Quaternion::from_axis_angle(axis, deg) * orientation_;
        orientation_.normalize();
    }

    // 均匀缩放，倍率限制在 [kMinScale, kMaxScale]，防止缩没了或撑爆画面
    void scale_by(float factor) {
        scale_ = std::min(std::max(scale_ * factor, kMinScale), kMaxScale);
    }

    // 背面剔除开关（B 键）。开启 = 只画朝向相机的面（"实体盒"），
    // 关闭 = 前后面都画（"玻璃盒"，能看到全部 12 条棱）。
    // 注意：剔除只解决"面自己朝向"，不解决面与面之间的遮挡——那是消隐。
    void enable_culling(bool on) { culling_ = on; }
    bool culling_enabled() const { return culling_; }

    // 阴影贴图开关（C 键）。关闭后所有面全量受光，用于对比观察阴影效果
    void enable_shadows(bool on) { shadows_ = on; }
    bool shadows_enabled() const { return shadows_; }

    // 光源视角深度 pass（公开给回归测试直接驱动；draw 内部以阴影开/关调用）。
    // sphere_model 只作用于模型球，默认恒等（地面固定不动，无模型变换）。
    void render_depth(FrameBuffer& target, const Mat4& light_vp,
                      const Mat4& sphere_model = Mat4()) const {
        render_depth_impl(target, light_vp, sphere_model);
    }

    // 几何只读访问（调试/回归测试用）
    static const std::vector<Vector3>& verts() { return kGeom.verts; }
    static const std::vector<std::vector<int>>& faces() { return kGeom.faces; }
    // 场景构成（回归测试用）：模型/地面的面与顶点分界、是否来自 OBJ
    static int model_face_count() { return kGeom.model_face_count; }
    static int model_vert_count() { return kGeom.model_vert_count; }
    static bool loaded_from_obj() { return kGeom.from_obj; }
    // 内部阴影图只读访问（调试/回归测试用）
    const FrameBuffer& shadow_map() const { return shadow_fb_; }

    // 只读访问（调试打印状态用）
    const Vector3& position() const { return position_; }
    const Quaternion& orientation() const { return orientation_; }
    float scale() const { return scale_; }

    // 走一遍渲染管线：阴影贴图 → 剔除 → 逐像素着色 → 近平面裁剪 → 投影
    //                → W-A 窗口裁剪 → AET 填充+消隐 → 描边
    void draw(FrameBuffer& fb, const ClipWindow& window, const PointLight& light,
              const Camera& cam) const {
        fb.clear();
        const Mat4 model = model_matrix();   // 模型球变换（地面不使用）

        // ---------- 管线 0：阴影贴图（光源视角深度 pass）----------
        // 光源看向场景中心，fov 100°（光源越远覆盖越大）；光源方向几乎
        // 竖直时 look_at 的 up 会退化，此时换 z 轴做参考向上向量。
        const Vector3 light_fwd = (Vector3(0, 0, 0) - light.position).normalized();
        const Vector3 light_up = (std::fabs(light_fwd.y) > 0.95f)
            ? Vector3(0, 0, 1.0f) : Vector3(0, 1.0f, 0);
        Mat4 lview = Mat4::look_at(light.position, Vector3(0, 0, 0), light_up);
        // 光源视锥自适应收紧：只需罩住投射阴影的模型球（地面不参与光源
        // 深度图）。固定 100° fov 在远光源时会把球压成深度图里几十像素的
        // 小盘，面片比纹素还小，薄片边守卫一跳过就漏光——实测影子区出现
        // 亮色锥形漏光。fov 按光源距离收紧后阴影分辨率大幅提高。
        const float ldist = light.position.length();
        const float lfov = std::min(120.0f, std::max(20.0f,
                2.0f * 57.29578f * std::atan(3.0f / std::max(ldist, 0.5f))));
        Mat4 lproj = Mat4::perspective(lfov, 1.0f, kLightNear, kLightFar);
        const Mat4 lvp = lproj * lview;
        const float kC_l = lproj.m[10], kE_l = lproj.m[11];     // NDC 反解视深用
        shadow_fb_.clear();
        if (shadows_) render_depth(shadow_fb_, lvp, model);

        // ---------- 管线 1-2：矩阵准备 + 顶点变换到裁剪空间 ----------
        const Vector3 eye = cam.position;   // 相机位置（可移动）
        Mat4 view = Mat4::look_at(eye, eye + cam.forward(), Vector3(0, 1, 0));
        Mat4 proj = Mat4::perspective(60.0f, fb.width() / float(fb.height()), 0.1f, 100.0f);
        const Mat4 vp  = proj * view;                     // 光源标记用（不随模型旋转）
        const Mat4 mvp = vp * model;                      // operator* 自右向左作用

        // 视线空间正交基（与上面 look_at 的构造严格一致）：逐像素着色时用它把
        // "屏幕坐标 + NDC 深度"重建出的视线空间点搬回世界空间。
        // 视线方向必须取 cam.forward()——此前误用"相机指向世界原点"的方向，
        // 默认机位恰好盯着原点、两方向重合掩盖了错误；相机一旦移动/旋转，
        // 重建出的世界坐标整体错位，阴影和光照便随视点游动（实测 bug，
        // 回归测试 test_shadow_stability.cpp 守护）。
        const Vector3 view_forward = cam.forward();
        const Vector3 view_right   = view_forward.cross(Vector3(0, 1.0f, 0)).normalized();
        const Vector3 view_up      = view_right.cross(view_forward).normalized();
        // 透视矩阵参数：z_ndc = -C + E/w  →  视深 w = E / (z_ndc + C)
        const float kC = proj.m[10], kE = proj.m[11];

        // 顶点变换分离：模型矩阵只作用于可变换模型；地面固定不动（直接用 vp）。
        // 此前地面跟着键盘变换一起动——缩放时地面缩成一小块漂在空中，
        // 移动时地面会抬到光源附近把光挡住，整片地面误影全是结构性问题。
        std::vector<Vec4> model_clip(kGeom.model_vert_count);
        for (int i = 0; i < kGeom.model_vert_count; ++i)
            model_clip[i] = mvp.transform_point_clip(kGeom.verts[i]);
        std::vector<Vec4> ground_clip(kGeom.verts.size() - size_t(kGeom.model_vert_count));
        for (size_t i = 0; i < ground_clip.size(); ++i)
            ground_clip[i] = vp.transform_point_clip(kGeom.verts[kGeom.model_vert_count + i]);

        // ---------- 管线 3-9：剔除 → 着色 → 裁剪 → 投影 → 窗口裁剪 → 填充+消隐 ----------
        const float W_NEAR = 0.05f;                        // 近平面（w = -z_view）
        const Rect win = window.rect(fb);

        // 内置回退配色：模型球暖红，地面石板灰（OBJ 导入时材质覆盖它们）
        const Vector3 sphere_albedo(0.85f, 0.35f, 0.30f);
        const Vector3 ground_albedo(0.45f, 0.48f, 0.52f);
        const bool obj_mode = kGeom.from_obj;
        const Vector3 sphere_c = model.transform_point(kGeom.center);   // 球心（世界）

        for (size_t fi = 0; fi < kGeom.faces.size(); ++fi) {
            const bool is_model = (int(fi) < kGeom.model_face_count);
            const auto& face = kGeom.faces[fi];
            const Mat4& m = is_model ? model : Mat4();          // 地面：恒等变换
            const auto& cbuf = is_model ? model_clip : ground_clip;
            const int vbase = is_model ? 0 : kGeom.model_vert_count;

            // ① 背面剔除（世界空间）：绕序朝外的面，法线指向观察者 = 正面。
            //    法线不必归一化，判符号即可；旋转由 model 矩阵带进法线里。
            const Vector3 w0 = m.transform_point(kGeom.verts[face[0]]);
            const Vector3 w1 = m.transform_point(kGeom.verts[face[1]]);
            const Vector3 w2 = m.transform_point(kGeom.verts[face[2]]);
            Vector3 normal = (w1 - w0).cross(w2 - w0);
            if (culling_ && normal.dot(eye - w0) <= 0.0f) continue;

            // ② 着色准备：反照率/高光/纹理。面挂了材质（OBJ 的 usemtl）：
            //    有 map_Kd 用纹理（Kd 存在时调制它），否则用 Kd 纯色；
            //    无材质用内置回退配色。
            Vector3 albedo = is_model ? sphere_albedo : ground_albedo;
            Vector3 spec_color = light.specular;
            float shininess = light.shininess;
            const Texture* tex = nullptr;
            Vector3 kd_mul(1, 1, 1);                        // 纹理的 Kd 调制系数
            if (size_t(fi) < kGeom.face_mat.size() && kGeom.face_mat[fi] >= 0) {
                const size_t mi = size_t(kGeom.face_mat[fi]);
                const Material& mt = kGeom.materials[mi];
                if (mi < kGeom.material_textures.size() && kGeom.material_textures[mi].ok()) {
                    tex = &kGeom.material_textures[mi];
                    if (mt.has_kd) kd_mul = mt.kd;          // MTL 语义：map_Kd ⊙ Kd
                } else {
                    albedo = mt.kd;                         // 无纹理：退回 Kd 纯色
                }
                spec_color = light.specular * mt.ks;        // 光源镜面 ⊙ 材质镜面率
                shininess = std::min(std::max(mt.shininess, 1.0f), 512.0f);
            }
            const Vector3 line_color = albedo * 0.4f;       // 深一号的同色描边
            bool flipped = false;
            if (normal.dot(eye - w0) < 0.0f) { normal = -normal; flipped = true; }
            normal = normal.normalized();

            // ②.5 角点着色属性：法线（模型空间 → 世界空间）与 UV。
            //     法线 transform_vector 不带平移；均匀缩放只改长度，shader
            //     里归一化抵消；背面翻转与面法线同步。无数据的回退场景用
            //     面法线（平面着色）、UV 用 (0,0)。
            Vector3 cn[4];
            Vec2 cuv[4];
            for (size_t k = 0; k < face.size() && k < 4; ++k) {
                cn[k] = (size_t(fi) < kGeom.face_normal.size())
                            ? m.transform_vector(kGeom.face_normal[fi][k])
                            : normal;
                cuv[k] = (size_t(fi) < kGeom.face_uv.size())
                             ? kGeom.face_uv[fi][k] : Vec2();
            }
            if (flipped)
                for (size_t k = 0; k < face.size() && k < 4; ++k) cn[k] = -cn[k];

            // 逐像素着色器：屏幕坐标 + NDC 深度 → 精确重建片元世界坐标 →
            // 光源空间采样阴影贴图（片元比遮挡物远 → 处于阴影，只剩环境光）
            // → 用真实世界坐标 + 插值法线 + 采样纹理算 Blinn-Phong。
            // 法线/UV 插值来自管线（UV 是透视校正的——(u/w,1/w) 屏幕仿射、
            // 像素处相除，见 PolyVert 注释）。
            auto shader = [&](float sx, float sy, float z01, const Vector3& n_interp,
                              float u, float v) -> Vector3 {
                const float ndc_x = sx / fb.width() * 2.0f - 1.0f;
                const float ndc_y = 1.0f - sy / fb.height() * 2.0f;
                const float w = kE / (2.0f * z01 - 1.0f + kC);  // 反解视深
                const Vector3 world = eye + view_right * (ndc_x * w / proj.m[0])
                                          + view_up * (ndc_y * w / proj.m[5])
                                          + view_forward * w;
                // 内置球仍用解析法线 normalize(world - 球心)：比插值更精确
                const Vector3 N = (!obj_mode && is_model)
                    ? (world - sphere_c).normalized() : n_interp.normalized();
                const Vector3 base = tex ? sample_texture(*tex, u, v) * kd_mul : albedo;
                const Vector3 lit = blinn_phong(world, N, base, spec_color,
                                                shininess, light, eye);
                if (!shadows_) return lit;

                const Vec4 lc = lvp.transform_point_clip(world);
                if (lc.w <= 0.0f) return lit;                   // 光源身后：按受光处理
                const float inv = 1.0f / lc.w;
                const int su = int(((lc.x * inv) + 1.0f) * 0.5f * kShadowRes);
                const int sv = int((1.0f - (lc.y * inv)) * 0.5f * kShadowRes);
                if (su < 0 || su >= kShadowRes || sv < 0 || sv >= kShadowRes) return lit;
                const float stored01 = shadow_fb_.depth_at(su, sv);
                if (stored01 > 1.0f) return lit;                // 深度图空洞：无遮挡物
                // 把深度图里的 NDC 反解回光源视深（线性量纲），再用世界单位
                // 偏置比较——片元侧 lc.w 本身就是光源视深。NDC 域取常数偏置
                // 会随 w² 失真（实测光源移远后影子飘出 1 个世界单位）。
                // 偏置再按入射角斜率缩放：表面与光线越接近平行，同一纹素内
                // 深度变化越大，需要越大的偏置压住自阴影痤疮。
                const Vector3 ldir = (light.position - world).normalized();
                const float ndl = std::fabs(N.dot(ldir));
                const float tan_theta = std::sqrt(std::max(0.0f, 1.0f - ndl * ndl))
                                      / std::max(ndl, 0.25f);
                const float bias = kShadowBiasW * (1.0f + 0.5f * tan_theta);
                const float stored_w = kE_l / (2.0f * stored01 - 1.0f + kC_l);
                if (lc.w > stored_w + bias)
                    return base * light.ambient;                // 阴影中：只剩环境光
                return lit;
            };

            // ③ 视锥体裁剪：近平面 + NDC 保护带（防止相机身后的面产生
            //    天文坐标，见 clip_polygon_frustum 的注释）；法线/UV 随顶点插值
            std::vector<ClipVert> quad;
            quad.reserve(face.size());
            for (size_t k = 0; k < face.size() && k < 4; ++k) {
                const int ix = face[k];
                quad.push_back({cbuf[ix - vbase], cn[k], cuv[k]});
            }
            std::vector<ClipVert> clipped = clip_polygon_frustum_n(quad, W_NEAR);
            if (clipped.size() < 3) continue;              // 整个面都在相机身后

            // ④ 齐次除法 + 视口变换 → 屏幕空间多边形（z = NDC 深度）。
            //    UV 从"裁剪空间的 (u,v)"换算成"屏幕仿射的 (u/w, v/w) + 1/w"，
            //    后续插值才满足透视校正（PolyVert 注释）
            std::vector<PolyVert> poly;
            poly.reserve(clipped.size());
            for (const ClipVert& c : clipped) {
                const float inv = 1.0f / c.c.w;
                poly.push_back({project_to_screen(c.c, fb), c.n,
                                c.uv * inv, inv});
            }

            // ⑤ Weiler–Atherton：把面多边形裁剪进矩形窗口（法线随环携带）
            for (const auto& loop : weiler_atherton(poly, win)) {
                // ⑥ AET 扫描线填充 + z-buffer 消隐：
                //    每个像素带插值深度参与测试，更近的片元才写入——
                //    与面的绘制顺序无关，凹体自遮挡也能得到正确画面；
                //    同时把面的编号写进识别缓冲，作为描边的归属凭据。
                fb.fill_polygon(loop, int(fi), shader);

                // ⑦ DDA 描出多边形轮廓：只画在自己面的填充赢下的像素上
                //    （id 门槛），被别的面挡住的描边自动不可见。
                //    仅内置回退场景描边（地面轮廓）；OBJ 网格面数多，
                //    三角化对角线会把画面涂碎，等三角形+重心坐标阶段重做
                if (!obj_mode && !is_model)
                    for (size_t i = 0; i < loop.size(); ++i) {
                        const Vector3& a = loop[i].pos;
                        const Vector3& b = loop[(i + 1) % loop.size()].pos;
                        fb.draw_line(a.x, a.y, b.x, b.y, line_color, int(fi));
                    }
            }
        }

        // ---------- 光源小球：在光源坐标处渲染一颗自发光小球 ----------
        // 模型矩阵只有平移 + 等比缩放（球各向同性，无需旋转）；
        // 走与场景面完全相同的裁剪/投影/W-A/填充管线，正常被 z-buffer 消隐，
        // 但不进阴影贴图、不算光照——它就是"灯泡"本身。
        {
            const Mat4 sphere_mvp = vp * (Mat4::translate(light.position.x, light.position.y,
                                                          light.position.z) *
                                          Mat4::scale(kLightSphereR, kLightSphereR,
                                                      kLightSphereR));
            const auto& sv = light_sphere().verts;
            const auto& sf = light_sphere().faces;
            std::vector<Vec4> sclip(sv.size());
            for (size_t i = 0; i < sv.size(); ++i)
                sclip[i] = sphere_mvp.transform_point_clip(sv[i]);
            const Vector3 emissive(1.0f, 0.85f, 0.25f);       // 自发光暖黄
            int sid = kSphereIdBase;
            for (const auto& face : sf) {
                std::vector<Vec4> quad;
                quad.reserve(face.size());
                for (int ix : face) quad.push_back(sclip[ix]);
                std::vector<Vec4> clipped = clip_polygon_frustum(quad, W_NEAR);
                if (clipped.size() < 3) continue;
                std::vector<PolyVert> poly;
                for (const Vec4& c : clipped) {
                    const float inv = 1.0f / c.w;
                    poly.push_back({project_to_screen(c, fb), Vector3(0, 0, 0),
                                    Vec2(), inv});
                }
                for (const auto& loop : weiler_atherton(poly, win))
                    fb.fill_polygon(loop, sid++, [emissive](float, float, float,
                                                            const Vector3&, float, float) {
                        return emissive;                      // 无描边：保持球面光滑
                    });
            }
        }

        // 点光源标记：光源位置投影到屏幕画一个黄色小十字。
        // overlay 绘制（不写深度/归属），无论光源藏到物体后面都可见，
        // 方便对照按键移动；光源在相机身后时跳过。
        const Vec4 lc = vp.transform_point_clip(light.position);
        if (lc.w > 0.1f) {
            const Vector3 ls = project_to_screen(lc, fb);
            const int cx = int(ls.x), cy = int(ls.y);
            const Vector3 marker(1.0f, 0.9f, 0.2f);
            for (int r = -5; r <= 5; ++r) {
                fb.plot(cx + r, cy, marker);
                fb.plot(cx, cy + r, marker);
            }
        }

        window.draw(fb);   // 最后画窗口框（overlay，不参与消隐，始终可见）
    }

private:
    // ============================================================
    // 场景几何：悬浮模型球 + 地面平面。
    // 抽成类级常量是因为阴影贴图 pass（render_depth）与主 pass
    // 必须渲染同一份几何。
    // 模型球：半径 1、球心 (0, 0.4, 0) → 底部离地 0.92，悬空便于
    // 观察完整的球形阴影；球面法线在着色时解析计算（逐像素平滑）。
    // 地面：12×12 平面，承接阴影。
    // ============================================================
    struct MeshData {
        std::vector<Vector3> verts;
        std::vector<std::vector<int>> faces;   // 顶点索引环（模型面是三角形，地面可能是四边形）
        // 与 faces 平行的角点着色法线（模型空间，来自 OBJ 的 vn；vn 缺失的
        // 面三个角点都用几何面法线）。逐像素插值它们 = 平滑着色（Phong 着色）；
        // 平面着色是"三个角点相同"的特例——法线数据自己决定外观。
        std::vector<std::array<Vector3, 3>> face_normal;
        // 与 faces 平行的角点 UV（模型空间，来自 OBJ 的 vt；缺失用 (0,0)），
        // 经透视校正插值后供纹理采样
        std::vector<std::array<Vec2, 3>> face_uv;
        std::vector<int> face_mat;             // 与 faces 平行的材质编号（-1 = 无材质）
        std::vector<Material> materials;       // OBJ 的材质表（回退场景为空）
        std::vector<Texture> material_textures;// 与 materials 平行；map_Kd 读入的像素
        int model_face_count = 0;              // 前 N 个面属于可变换模型，其余是固定地面
        int model_vert_count = 0;              // 前 N 个顶点属于模型
        bool from_obj = false;                 // 是否来自 OBJ 导入（决定法线/描边/配色行为）
        Vector3 center{0, 0, 0};               // 内置球球心（仅回退路径的解析法线用）
        int sphere_face_count = 0;             // 内置球面数（仅回退路径）
        int sphere_vert_count = 0;             // 内置球顶点数（仅回退路径）
    };

    // 生成经纬分片球：stacks 个纬带 × sectors 个经度片，极点三角形收口，
    // 绕序统一"从球外看逆时针"（法线朝外）。光源标记球与回退球复用。
    static MeshData make_sphere(int stacks, int sectors, float radius,
                                const Vector3& center) {
        MeshData g;
        g.center = center;
        g.verts.reserve(2 + size_t(stacks - 1) * sectors);
        g.verts.push_back({center.x, center.y + radius, center.z});      // 北极
        for (int s = 1; s < stacks; ++s) {
            const float phi = 3.14159265f * float(s) / float(stacks);
            for (int t = 0; t < sectors; ++t) {
                const float th = 6.28318530f * float(t) / float(sectors);
                g.verts.push_back({center.x + radius * std::sin(phi) * std::cos(th),
                                   center.y + radius * std::cos(phi),
                                   center.z + radius * std::sin(phi) * std::sin(th)});
            }
        }
        g.verts.push_back({center.x, center.y - radius, center.z});      // 南极
        auto ring = [&](int s, int t) { return 1 + (s - 1) * sectors + (t % sectors); };
        for (int t = 0; t < sectors; ++t)
            g.faces.push_back({0, ring(1, t + 1), ring(1, t)});
        for (int s = 1; s + 1 < stacks; ++s)
            for (int t = 0; t < sectors; ++t)
                g.faces.push_back({ring(s, t), ring(s, t + 1),
                                   ring(s + 1, t + 1), ring(s + 1, t)});
        const int south = 1 + (stacks - 1) * sectors;
        for (int t = 0; t < sectors; ++t)
            g.faces.push_back({south, ring(stacks - 1, t), ring(stacks - 1, t + 1)});
        return g;
    }

    // ============================================================
    // 场景装配：OBJ 导入 → 内置回退（见类注释的规则）。
    // 程序启动后 kGeom 静态初始化时执行一次，此后每帧只读。
    // ============================================================
    static MeshData build_scene() {
        // ---- 回退场景：内置悬浮球 + 30×30 地面（与 OBJ 无关，永远先备好）----
        MeshData g = make_sphere(18, 24, 1.0f, Vector3(0, 0.4f, 0));
        g.sphere_face_count = int(g.faces.size());
        g.sphere_vert_count = int(g.verts.size());
        const int base = int(g.verts.size());
        g.verts.push_back({-15, -1.52f, -15});
        g.verts.push_back({-15, -1.52f, 15});
        g.verts.push_back({15, -1.52f, 15});
        g.verts.push_back({15, -1.52f, -15});
        g.faces.push_back({base, base + 1, base + 2, base + 3});
        g.model_face_count = g.sphere_face_count;
        g.model_vert_count = g.sphere_vert_count;

        // ---- ① 导入 OBJ（相对工作目录的 src/ 优先，其次 asset/，再兜底上级目录）----
        ObjModel obj;
        const char* used_path = nullptr;
        for (const char* p : {"src/ball.obj", "../src/ball.obj",
                              "asset/ball.obj", "../asset/ball.obj"}) {
            std::string err;
            if (load_obj(p, obj, &err)) { used_path = p; break; }
        }
        if (!obj.ok) {
            std::printf("[scene] OBJ 导入失败（尝试过 src/ 与 asset/），回退内置球场景\n");
            return g;
        }

        // ---- ② 选固定地面：水平足迹最大且扁平的对象 ----
        //      扁平 = y 向尺寸 < 10%·max(x, z 向尺寸)。这能稳稳挑出
        //      "地面/台板"类对象，又不会误伤球、立方体等测试体。
        const int nobj = int(obj.objects.size());
        std::vector<Vector3> omin(size_t(nobj), {1e9f, 1e9f, 1e9f});
        std::vector<Vector3> omax(size_t(nobj), {-1e9f, -1e9f, -1e9f});
        for (const ObjFace& f : obj.faces) {
            if (f.object < 0 || f.object >= nobj) continue;
            for (int k = 0; k < 3; ++k) {
                const Vector3& p = obj.verts[size_t(f.v[k])];
                Vector3& lo = omin[size_t(f.object)];
                Vector3& hi = omax[size_t(f.object)];
                lo.x = std::min(lo.x, p.x); lo.y = std::min(lo.y, p.y); lo.z = std::min(lo.z, p.z);
                hi.x = std::max(hi.x, p.x); hi.y = std::max(hi.y, p.y); hi.z = std::max(hi.z, p.z);
            }
        }
        int ground_obj = -1;
        float best_fp = 0.0f;
        for (int o = 0; o < nobj; ++o) {
            const Vector3 ext = omax[size_t(o)] - omin[size_t(o)];
            if (ext.x <= 0.0f || ext.z <= 0.0f) continue;      // 空对象
            const bool flat = ext.y < 0.1f * std::max(ext.x, ext.z);
            const float fp = ext.x * ext.z;                    // 水平足迹
            if (flat && fp > best_fp) { best_fp = fp; ground_obj = o; }
        }

        // ---- ③ 重组顶点/面：模型面在前（0 基重映射），地面面在后 ----
        //      模型与地面各用一套重映射表——同一个顶点若被两边共用，
        //      也会拆成两个副本，保证两段顶点区间互不越界。
        MeshData s;
        s.from_obj = true;
        s.materials = obj.materials;
        // 材质纹理：map_Kd 的路径已由 mtlloader 解析好，这里读入像素数据
        //（贴图阶段的核心一步——此前只记路径不读图）
        s.material_textures.reserve(s.materials.size());
        for (const Material& m : s.materials)
            s.material_textures.push_back(m.map_kd.empty()
                                              ? Texture() : load_texture(m.map_kd));
        std::vector<int> remap_m(obj.verts.size(), -1), remap_g(obj.verts.size(), -1);
        // 角点法线解析：有 vn 用 vn；缺失的角点回退到几何面法线。
        // 注意不按 s 平滑标记分流——外观由 vn 数据本身决定：Blender 平直
        // 着色导出的 vn 三个角点相同（插值后仍是平面着色），Shade Smooth
        // 导出的 vn 逐顶点渐变（插值后平滑），一套插值机制通吃两者。
        const auto corner_normals = [&](const ObjFace& f) {
            std::array<Vector3, 3> ns;
            const Vector3 gn = (obj.verts[size_t(f.v[1])] - obj.verts[size_t(f.v[0])])
                       .cross(obj.verts[size_t(f.v[2])] - obj.verts[size_t(f.v[0])]);
            for (int k = 0; k < 3; ++k)
                ns[k] = (f.n[k] >= 0) ? obj.normals[size_t(f.n[k])] : gn;
            return ns;
        };
        // 角点 UV 解析：有 vt 用 vt；缺失的角点用 (0,0)
        const auto corner_uvs = [&](const ObjFace& f) {
            std::array<Vec2, 3> ts;
            for (int k = 0; k < 3; ++k)
                ts[k] = (f.t[k] >= 0) ? Vec2(obj.uvs[size_t(f.t[k])].x,
                                             obj.uvs[size_t(f.t[k])].y) : Vec2();
            return ts;
        };
        const auto collect = [&](bool want_model, std::vector<int>& remap) {
            for (const ObjFace& f : obj.faces) {
                const bool is_ground = (ground_obj >= 0 && f.object == ground_obj);
                if (is_ground == want_model) continue;         // 本轮只要另一类
                std::vector<int> idx;
                idx.reserve(3);
                for (int k = 0; k < 3; ++k) {
                    const size_t old = size_t(f.v[k]);
                    if (remap[old] < 0) {
                        remap[old] = int(s.verts.size());
                        s.verts.push_back(obj.verts[old]);
                    }
                    idx.push_back(remap[old]);
                }
                s.faces.push_back(idx);
                s.face_normal.push_back(corner_normals(f));
                s.face_uv.push_back(corner_uvs(f));
                s.face_mat.push_back(f.material);
            }
        };
        collect(true, remap_m);
        s.model_face_count = int(s.faces.size());
        s.model_vert_count = int(s.verts.size());
        collect(false, remap_g);
        if (s.model_face_count == 0) {
            std::printf("[scene] OBJ 里没有模型对象（全是地面？），回退内置球场景\n");
            return g;
        }

        // ---- ④ 地面顶端高度：模型悬浮的基准面 ----
        float ground_top = -1.52f;                             // 内置地面的默认高度
        for (size_t i = size_t(s.model_vert_count); i < s.verts.size(); ++i)
            ground_top = std::max(ground_top, s.verts[i].y);
        if (ground_obj < 0) {                                  // OBJ 无地面：补内置地面
            const int b = int(s.verts.size());
            s.verts.push_back({-15, -1.52f, -15});
            s.verts.push_back({-15, -1.52f, 15});
            s.verts.push_back({15, -1.52f, 15});
            s.verts.push_back({15, -1.52f, -15});
            s.faces.push_back({b, b + 1, b + 2, b + 3});
            s.face_mat.push_back(-1);
        }

        // ---- ⑤ 模型归一化：等比缩放到最大边长 2，x/z 居中， ----
        //      底部抬到地面上方 kHoverGap（内置球摆位的悬浮间隙）
        constexpr float kHoverGap = 0.92f;   // 内置球底(-0.6) 距内置地面(-1.52) 的间隙
        Vector3 mn(1e9f, 1e9f, 1e9f), mx(-1e9f, -1e9f, -1e9f);
        for (int i = 0; i < s.model_vert_count; ++i) {
            const Vector3& p = s.verts[size_t(i)];
            mn.x = std::min(mn.x, p.x); mn.y = std::min(mn.y, p.y); mn.z = std::min(mn.z, p.z);
            mx.x = std::max(mx.x, p.x); mx.y = std::max(mx.y, p.y); mx.z = std::max(mx.z, p.z);
        }
        const float ext = std::max(mx.x - mn.x, std::max(mx.y - mn.y, mx.z - mn.z));
        const float sc = (ext > 1e-6f) ? 2.0f / ext : 1.0f;
        const float cx = 0.5f * (mn.x + mx.x), cz = 0.5f * (mn.z + mx.z);
        for (int i = 0; i < s.model_vert_count; ++i) {
            Vector3& p = s.verts[size_t(i)];
            p.x = (p.x - cx) * sc;
            p.z = (p.z - cz) * sc;
            p.y = (p.y - mn.y) * sc + ground_top + kHoverGap;
        }

        std::printf("[scene] 已导入 %s：%zu 面 / %zu 材质 / %zu 对象（地面=%s）\n",
                    used_path, s.faces.size(), s.materials.size(), obj.objects.size(),
                    ground_obj >= 0 ? obj.objects[size_t(ground_obj)].name.c_str() : "无，用内置");
        return s;
    }

    inline static const MeshData kGeom = build_scene();

    // 光源标记小球（单位球，渲染时矩阵缩放平移到光源处）
    static const MeshData& light_sphere() {
        static const MeshData m = make_sphere(6, 10, 1.0f, Vector3(0, 0, 0));
        return m;
    }

    // ---- 阴影贴图参数 ----
    static constexpr int kShadowRes = 512;        // 阴影贴图分辨率
    static constexpr float kLightNear = 0.1f;     // 光源投影近平面
    static constexpr float kLightFar = 120.0f;    // 光源投影远平面
    // 深度比较偏置（世界单位）。关键教训：偏置绝不能在 NDC 域取常数——
    // NDC 深度非线性，同一个 NDC 偏置对应的世界偏移随 w² 放大
    // （w=3 时 0.14、w=8 时 1.0 个世界单位，实测光源移远后影子"飘"）。
    // 比较时把深度图里的 NDC 反解回光源视深（线性），再用世界单位偏置。
    static constexpr float kShadowBiasW = 0.08f;

    // 以下内容为内部实现，但 render_depth 供回归测试直接驱动光源深度 pass
    // ============================================================
    // ---- 光源小球：光源位置的"实体标记" ----
    // 不参与阴影贴图 pass（若小球进了光源深度图，光源会被自己包住，
    // 全场皆影）；自发光恒定亮色，不受光照与阴影影响；
    // 正常参与 z-buffer（会被台阶挡住），十字 marker 最后 overlay，
    // 所以小球永远不会遮住光源标记。
    static constexpr float kLightSphereR = 0.12f;   // 小球半径（世界单位）
    static constexpr int kSphereIdBase = 1000;      // 小球面的 id 段（场景面 id 现已达 400+，留足间隔）

    // 光源视角深度 pass 实现：把全部面渲染进 target（阴影测试只读深度通道）。
    // 完全复用 AET 填充——把"光源 VP 下的屏幕多边形"填进去即可，颜色无所谓。
    void render_depth_impl(FrameBuffer& target, const Mat4& light_vp,
                           const Mat4& sphere_model) const {
        const Mat4 model_lvp = light_vp * sphere_model;
        for (size_t fi = 0; fi < kGeom.faces.size(); ++fi) {
            const bool is_model = (int(fi) < kGeom.model_face_count);
            // 地面只接收阴影、不投射阴影（它下方没有别的物体），因此不进
            // 光源深度图——若参与，掠射角下地面与自己写下的深度逐纹素比较，
            // 误差超过偏置就产生波纹状的阴影条纹（实测：光源移远后地面
            // 出现明暗相间的波纹带）。物体才是阴影的"投射者"。
            if (!is_model) continue;
            const auto& face = kGeom.faces[fi];
            std::vector<Vec4> quad;
            quad.reserve(face.size());
            for (int ix : face) quad.push_back(model_lvp.transform_point_clip(kGeom.verts[ix]));
            std::vector<Vec4> clipped = clip_polygon_frustum(quad, kLightNear);
            if (clipped.size() < 3) continue;      // 整个面都在光源身后
            std::vector<Vector3> poly;
            for (const Vec4& c : clipped) poly.push_back(project_to_screen(c, target));
            target.fill_polygon(poly, Vector3(0, 0, 0), int(fi));
        }
    }

    // 模型矩阵 = T · R · S（R 由四元数转出）
    // 作用顺序（从右往左）：先缩放，再旋转，最后平移。
    // 姿态只有一份（四元数）；"绕自身轴"与"绕世界轴"只是右乘/左乘之别，
    // 这正是真实引擎（Unity 的 Space.Self / Space.World）的实现方式。
    Mat4 model_matrix() const {
        return Mat4::translate(position_.x, position_.y, position_.z) *
               orientation_.to_matrix() *
               Mat4::scale(scale_, scale_, scale_);
    }

    static constexpr float kMinScale = 0.05f;
    static constexpr float kMaxScale = 20.0f;

    Vector3 position_{0, 0, 0};        // 世界系平移
    Quaternion orientation_;           // 姿态（单位四元数），默认不旋转
    float scale_ = 1.0f;               // 均匀缩放
    bool culling_ = true;              // 背面剔除开关
    bool shadows_ = true;              // 阴影贴图开关
    mutable FrameBuffer shadow_fb_{kShadowRes, kShadowRes};   // 光源深度图（跨帧复用）
};
