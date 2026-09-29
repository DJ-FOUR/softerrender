#pragma once
// ============================================================
// 数学库：向量、齐次坐标、4x4 矩阵
// 所有渲染用到的线性代数都在这里，不依赖任何图形库。
//
// C++ 风格说明：
//   - Vector3/Vec4/Mat4 用 struct：C++ 中 struct 就是 class（默认 public 而已），
//     "只有数据、无不变式"的简单值类型用 struct 是现代 C++ 惯用法（GLM 也如此）；
//   - 运算符重载让代码和数学公式同形：a+b、a·b、a×b、M*v、s*v；
//   - 重载 << 把向量直接打进 std::ostream，调试输出不用再拼格式串。
// ============================================================
#include <cmath>
#include <ostream>

// 二维向量：纹理坐标（UV）用
struct Vec2 {
    float x = 0, y = 0;
    Vec2() = default;
    Vec2(float x_, float y_) : x(x_), y(y_) {}
    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(float s) const { return {x * s, y * s}; }
    Vec2 operator/(float s) const { return {x / s, y / s}; }
    Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
    friend Vec2 operator*(float s, const Vec2& v) { return v * s; }
};

// 三维向量
struct Vector3 {
    float x = 0, y = 0, z = 0;
    Vector3() = default;
    Vector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vector3 operator+(const Vector3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vector3 operator-(const Vector3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vector3 operator-() const { return {-x, -y, -z}; }            // 取负向量
    Vector3 operator*(float s) const { return {x * s, y * s, z * s}; }
    // 逐分量乘：颜色调制用（albedo ⊙ 光强），与标量乘重载不冲突
    Vector3 operator*(const Vector3& o) const { return {x * o.x, y * o.y, z * o.z}; }
    Vector3 operator/(float s) const { return {x / s, y / s, z / s}; }

    Vector3& operator+=(const Vector3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vector3& operator-=(const Vector3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vector3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }

    // 标量在左的乘法：s * v（v * s 走上面的成员版本）
    friend Vector3 operator*(float s, const Vector3& v) { return v * s; }

    // 点积：a·b = |a||b|cosθ，用于 N·L 光照、背面判断、投影
    float dot(const Vector3& o) const { return x * o.x + y * o.y + z * o.z; }

    // 叉积：方向由右手定则决定，模长为平行四边形面积。
    // 用途：求平面法线、三角形有向面积（背面剔除/重心坐标）
    Vector3 cross(const Vector3& o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }

    float length() const { return std::sqrt(x * x + y * y + z * z); }
    Vector3 normalized() const {
        float l = length();
        return (l > 1e-8f) ? Vector3{x / l, y / l, z / l} : Vector3{0, 0, 0};
    }

    // 打印成 (x, y, z)
    friend std::ostream& operator<<(std::ostream& os, const Vector3& v) {
        return os << '(' << v.x << ", " << v.y << ", " << v.z << ')';
    }
};

// 齐次坐标（x,y,z,w）
// 约定：w=1 表示点（受平移影响），w=0 表示方向向量（不受平移影响）。
// 投影矩阵会把视深写入 w，齐次除法（x/w）由此产生透视"近大远小"。
struct Vec4 {
    float x = 0, y = 0, z = 0, w = 1;
};

// 4x4 矩阵，行主序：m[row * 4 + col]
// 矩阵作用于列向量：v' = M * v
struct Mat4 {
    float m[16] = {0};

    Mat4() { m[0] = m[5] = m[10] = m[15] = 1.0f; }  // 默认单位矩阵

    static Mat4 multiply(const Mat4& a, const Mat4& b) {
        Mat4 r;
        for (int row = 0; row < 4; ++row)
            for (int col = 0; col < 4; ++col) {
                float sum = 0;
                for (int k = 0; k < 4; ++k) sum += a.m[row * 4 + k] * b.m[k * 4 + col];
                r.m[row * 4 + col] = sum;
            }
        return r;
    }

    // 运算符写法：a * b 等价于 multiply(a, b)，作用顺序是先 b 后 a
    Mat4 operator*(const Mat4& o) const { return multiply(*this, o); }

    // 平移矩阵：把点沿各轴移动 (tx, ty, tz)。平移量写在第四列，
    // 只对 w=1 的点起作用——这正是齐次坐标区分"点/方向"的意义。
    static Mat4 translate(float tx, float ty, float tz) {
        Mat4 r;
        r.m[3] = tx; r.m[7] = ty; r.m[11] = tz;
        return r;
    }

    // 缩放矩阵（各轴独立；均匀缩放三个参数传同一个值）
    static Mat4 scale(float sx, float sy, float sz) {
        Mat4 r;
        r.m[0] = sx; r.m[5] = sy; r.m[10] = sz;
        return r;
    }

    // 绕 Y 轴旋转（角度制）。三维旋转的通用形式是罗德里格斯公式，
    // 这里给出绕三个坐标轴的版本，任意朝向可由它们组合出来。
    static Mat4 rotate_y(float deg) {
        Mat4 r;
        float rad = deg * 3.14159265f / 180.0f;
        float c = std::cos(rad), s = std::sin(rad);
        r.m[0] = c;  r.m[2] = s;
        r.m[8] = -s; r.m[10] = c;
        return r;
    }

    // 绕 X 轴旋转（角度制）：y、z 在各自平面内旋转（俯仰）
    static Mat4 rotate_x(float deg) {
        Mat4 r;
        float rad = deg * 3.14159265f / 180.0f;
        float c = std::cos(rad), s = std::sin(rad);
        r.m[5] = c;  r.m[6] = -s;
        r.m[9] = s;  r.m[10] = c;
        return r;
    }

    // 绕 Z 轴旋转（角度制）：x、y 在各自平面内旋转（滚转）
    static Mat4 rotate_z(float deg) {
        Mat4 r;
        float rad = deg * 3.14159265f / 180.0f;
        float c = std::cos(rad), s = std::sin(rad);
        r.m[0] = c;  r.m[1] = -s;
        r.m[4] = s;  r.m[5] = c;
        return r;
    }

    // 视图矩阵 = 相机世界变换的逆：用 forward/right/up 正交基 + 平移构造
    static Mat4 look_at(const Vector3& eye, const Vector3& target, const Vector3& up) {
        Vector3 forward = (target - eye).normalized();   // 相机看 -z，所以第三行取负
        Vector3 right   = forward.cross(up).normalized();
        Vector3 new_up  = right.cross(forward).normalized();

        Mat4 r;
        r.m[0] = right.x;   r.m[1] = right.y;   r.m[2] = right.z;
        r.m[4] = new_up.x;  r.m[5] = new_up.y;  r.m[6] = new_up.z;
        r.m[8] = -forward.x; r.m[9] = -forward.y; r.m[10] = -forward.z;
        // 平移列：-R * eye
        r.m[3]  = -right.dot(eye);
        r.m[7]  = -new_up.dot(eye);
        r.m[11] = forward.dot(eye);
        return r;
    }

    // 透视投影矩阵：把视锥体压成 NDC 的 [-1,1]^3 立方体。
    // 关键设计：w_clip = -z_view，即把视深编码进 w，
    // 齐次除法后自动完成近大远小；z 的映射是非线性的（近处精度高）。
    static Mat4 perspective(float fov_deg, float aspect, float zn, float zf) {
        Mat4 r;
        float f = 1.0f / std::tan(fov_deg * 3.14159265f / 360.0f);  // = 1/tan(fov/2)
        r.m[0]  = f / aspect;
        r.m[5]  = f;
        r.m[10] = (zf + zn) / (zn - zf);
        r.m[11] = 2.0f * zf * zn / (zn - zf);
        r.m[14] = -1.0f;
        r.m[15] = 0.0f;
        return r;
    }

    // 仿射变换一个点（w=1，不产生透视），用于 model/view 阶段
    Vector3 transform_point(const Vector3& p) const {
        return {
            m[0] * p.x + m[1] * p.y + m[2] * p.z + m[3],
            m[4] * p.x + m[5] * p.y + m[6] * p.z + m[7],
            m[8] * p.x + m[9] * p.y + m[10] * p.z + m[11],
        };
    }

    // 变换一个方向向量（不加分量的平移列）——法线变换用。
    // 注意：严格做法是非均匀缩放下用逆转置矩阵；本引擎缩放恒为均匀，
    // 旋转+均匀缩放作用在法线上只需变换后重新归一化即可。
    Vector3 transform_vector(const Vector3& v) const {
        return {
            m[0] * v.x + m[1] * v.y + m[2] * v.z,
            m[4] * v.x + m[5] * v.y + m[6] * v.z,
            m[8] * v.x + m[9] * v.y + m[10] * v.z,
        };
    }

    // 完整齐次变换：点(w=1) → 裁剪空间坐标（x,y,z 未除 w）
    Vec4 transform_point_clip(const Vector3& p) const {
        return {
            m[0] * p.x + m[1] * p.y + m[2] * p.z + m[3],
            m[4] * p.x + m[5] * p.y + m[6] * p.z + m[7],
            m[8] * p.x + m[9] * p.y + m[10] * p.z + m[11],
            m[12] * p.x + m[13] * p.y + m[14] * p.z + m[15],
        };
    }
};

// ============================================================
// 四元数 (w, x, y, z)：三维旋转的标准姿态存储格式（引擎通用做法）。
// 相比欧拉角：无万向节锁、不存在"分量对应哪个坐标系"的歧义；
// 相比旋转矩阵：参数少、漂移慢、归一化便宜。
//
// 约定（与 Mat4 一致）：
//   - 单位四元数 q 把局部方向旋到世界方向；
//   - 乘法 q1·q2（哈密顿积）表示"先施加 q2，再施加 q1"，
//     于是 局部旋转 = 右乘 q·Δq，全局旋转 = 左乘 Δq·q —— 同一份姿态，两种乘法。
// ============================================================
struct Quaternion {
    float w = 1, x = 0, y = 0, z = 0;    // 默认 = 单位四元数（不旋转）

    Quaternion() = default;
    Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    // 绕单位轴 axis 转 deg 度：q = (cos(θ/2), axis·sin(θ/2))。
    // 半角 θ/2 是四元数特有的；这是罗德里格斯旋转公式的四元数形式。
    static Quaternion from_axis_angle(const Vector3& axis, float deg) {
        const float rad = deg * 3.14159265f / 180.0f;
        const float half = rad * 0.5f;
        const float s = std::sin(half);
        const Vector3 a = axis.normalized();
        return {std::cos(half), a.x * s, a.y * s, a.z * s};
    }

    // 哈密顿积
    Quaternion operator*(const Quaternion& o) const {
        return {w * o.w - x * o.x - y * o.y - z * o.z,
                w * o.x + x * o.w + y * o.z - z * o.y,
                w * o.y - x * o.z + y * o.w + z * o.x,
                w * o.z + x * o.y - y * o.x + z * o.w};
    }

    Quaternion conjugate() const { return {w, -x, -y, -z}; }

    // 每次旋转后调用：浮点误差会让长度缓慢漂移，拉回单位长度
    void normalize() {
        float len = std::sqrt(w * w + x * x + y * y + z * z);
        if (len > 1e-12f) { w /= len; x /= len; y /= len; z /= len; }
    }

    // 旋转向量：v' = q·(0,v)·q* 的展开式（省去中间矩阵）。
    // 设 qv = (x,y,z)，则 v' = v + 2·w·(qv×v) + 2·qv×(qv×v)
    Vector3 rotate(const Vector3& v) const {
        Vector3 qv(x, y, z);
        return v + (qv.cross(v) * w + qv.cross(qv.cross(v))) * 2.0f;
    }

    // 转成 4x4 旋转矩阵（行主序、列向量约定），供建模矩阵级联。
    // 仅对单位四元数成立——调用方需保证已 normalize()。
    Mat4 to_matrix() const {
        Mat4 r;
        const float xx = x * x, yy = y * y, zz = z * z;
        const float xy = x * y, xz = x * z, yz = y * z;
        const float wx = w * x, wy = w * y, wz = w * z;
        r.m[0]  = 1.0f - 2.0f * (yy + zz);
        r.m[1]  =        2.0f * (xy - wz);
        r.m[2]  =        2.0f * (xz + wy);
        r.m[4]  =        2.0f * (xy + wz);
        r.m[5]  = 1.0f - 2.0f * (xx + zz);
        r.m[6]  =        2.0f * (yz - wx);
        r.m[8]  =        2.0f * (xz - wy);
        r.m[9]  =        2.0f * (yz + wx);
        r.m[10] = 1.0f - 2.0f * (xx + yy);
        return r;
    }

    // 打印成 q(w, x, y, z)
    friend std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
        return os << "q(" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << ')';
    }
};
