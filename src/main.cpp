// ============================================================
// 程序入口：SDL3 窗口 + 键盘交互 + GL_POINTS 上屏
//
// 分工（符合"纯 CPU 渲染，OpenGL 只画点"）：
//   App（本文件）—— 创建窗口、收键盘事件、交换缓冲、上屏
//   OpenGL       —— 只做一件事：把 CPU 算好的像素缓冲用 GL_POINTS 画出来
//   渲染          —— 100% 在 soft_renderer.h 中由 CPU 完成（当前：悬浮小球 + 地面）
//
// C++ 结构：
//   - App 用 RAII 管理资源：构造函数里初始化 SDL/OpenGL，析构函数自动清理，
//     任何路径退出（包括异常）都不会泄漏窗口和 GL 上下文；
//   - 键盘输入被翻译成 SolidMesh 的方法调用（move / rotate / scale_by），
//     输入层不直接碰实体的内部状态——封装边界在这。
// ============================================================
#include <GL/gl.h>          // MinGW 自带 OpenGL 1.1 头（含 glOrtho）
#include <SDL3/SDL.h>
#include <iostream>
#include "soft_renderer.h"

class App {
public:
    App() {
        SDL_Init(SDL_INIT_VIDEO);
        window_ = SDL_CreateWindow(
            "CPU Software Renderer - Floating Sphere (SDL3)", kW, kH, SDL_WINDOW_OPENGL);
        gl_ctx_ = SDL_GL_CreateContext(window_);
        SDL_GL_SetSwapInterval(1);                        // 垂直同步，锁 ~60fps

        // 2D 正交投影，配合画点（原点居中，y 向上）
        glViewport(0, 0, kW, kH);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(-kW / 2.0, kW / 2.0, -kH / 2.0, kH / 2.0, -1.0, 1.0);
        glMatrixMode(GL_MODELVIEW);

        print_help();
    }

    ~App() {
        SDL_GL_DestroyContext(gl_ctx_);
        SDL_DestroyWindow(window_);
        SDL_Quit();
    }

    // 主循环：事件轮询 → CPU 渲染 → 上屏
    void run() {
        bool running = true;
        while (running) {
            SDL_Event event;
            while (SDL_PollEvent(&event)) {
                if (event.type == SDL_EVENT_QUIT) running = false;
                else if (event.type == SDL_EVENT_KEY_DOWN) {
                    if (event.key.key == SDLK_ESCAPE) running = false;
                    else handle_key(event.key.key, event.key.mod); // 按住可连续触发
                }
            }

            solid_.draw(fb_, clip_window_, light_, camera_);     // ① 纯 CPU 渲染整幅图像（无输入时画面不变）
            glClear(GL_COLOR_BUFFER_BIT);
            draw_framebuffer();           // ② OpenGL 画点显示
            SDL_GL_SwapWindow(window_);   // ③ 交换缓冲
        }
    }

private:
    // 键盘分发：无修饰键 = 摄像机；Shift = 物体变换（世界轴）；
    // Ctrl = 物体旋转（自身轴）。SDL 按键不区分大小写，用修饰键实现
    // 旧项目"小写控物体、大写控相机"的分工。
    void handle_key(SDL_Keycode key, SDL_Keymod mods) {
        const float kMove = 0.25f;    // 每次平移的世界单位
        const float kRot  = 10.0f;    // 每次旋转的角度（度）
        const float kZoom = 1.1f;     // 每次缩放的倍率
        const float kLight = 0.25f;   // 光源每次移动的世界单位
        const bool shift = (mods & SDL_KMOD_SHIFT) != 0;
        const bool ctrl  = (mods & SDL_KMOD_CTRL)  != 0;
        const Vector3 xaxis(1, 0, 0), yaxis(0, 1, 0), zaxis(0, 0, 1);

        // 光源移动：不许挪进世界原点附近——光源 look_at 原点，
        // eye 与 target 重合时视图矩阵退化
        auto move_light = [&](const Vector3& d) {
            const Vector3 np = light_.position + d;
            if (np.length() > 0.25f) light_.position = np;
        };

        // ---- 无修饰键：摄像机 + 点光源 ----
        if (!shift && !ctrl) {
            switch (key) {
            case SDLK_W: camera_.move(camera_.forward() * kMove); break;   // 前进
            case SDLK_S: camera_.move(camera_.forward() * -kMove); break;  // 后退
            case SDLK_A: camera_.move(camera_.right() * -kMove); break;    // 左移
            case SDLK_D: camera_.move(camera_.right() * kMove); break;     // 右移
            case SDLK_Q: camera_.turn_yaw(-kRot); break;                   // 左摇头
            case SDLK_E: camera_.turn_yaw(kRot); break;                    // 右摇头
            case SDLK_R: camera_.turn_pitch(kRot); break;                  // 抬头
            case SDLK_F: camera_.turn_pitch(-kRot); break;                 // 低头
            // 点光源：I/K 下/上  J/L 左/右  U/O 靠近/远离（沿世界轴，README 键位）
            case SDLK_I: move_light({0, -kLight, 0}); break;
            case SDLK_K: move_light({0,  kLight, 0}); break;
            case SDLK_J: move_light({-kLight, 0, 0}); break;
            case SDLK_L: move_light({ kLight, 0, 0}); break;
            case SDLK_U: move_light({0, 0,  kLight}); break;
            case SDLK_O: move_light({0, 0, -kLight}); break;
            default: return;
            }
            print_status();
            return;
        }

        // ---- Shift / Ctrl：物体变换 ----
        switch (key) {
        // 移动（Shift；地面固定不动）
        case SDLK_W: if (shift) solid_.move({0,  kMove, 0}); break;    // 上移
        case SDLK_S: if (shift) solid_.move({0, -kMove, 0}); break;    // 下移
        case SDLK_A: if (shift) solid_.move({-kMove, 0, 0}); break;    // 左移
        case SDLK_D: if (shift) solid_.move({ kMove, 0, 0}); break;    // 右移
        case SDLK_N: if (shift) solid_.move({0, 0,  kMove}); break;    // 靠近相机
        case SDLK_M: if (shift) solid_.move({0, 0, -kMove}); break;    // 远离相机
        // 旋转：Shift = 绕世界轴，Ctrl = 绕自身轴
        case SDLK_T: if (shift) solid_.rotate_world(xaxis,  kRot);
                     if (ctrl)  solid_.rotate_local(xaxis,  kRot); break;
        case SDLK_G: if (shift) solid_.rotate_world(xaxis, -kRot);
                     if (ctrl)  solid_.rotate_local(xaxis, -kRot); break;
        case SDLK_R: if (shift) solid_.rotate_world(yaxis,  kRot);
                     if (ctrl)  solid_.rotate_local(yaxis,  kRot); break;
        case SDLK_F: if (shift) solid_.rotate_world(yaxis, -kRot);
                     if (ctrl)  solid_.rotate_local(yaxis, -kRot); break;
        case SDLK_Q: if (shift) solid_.rotate_world(zaxis,  kRot);
                     if (ctrl)  solid_.rotate_local(zaxis,  kRot); break;
        case SDLK_E: if (shift) solid_.rotate_world(zaxis, -kRot);
                     if (ctrl)  solid_.rotate_local(zaxis, -kRot); break;
        // 缩放（Shift）
        case SDLK_Z: if (shift) solid_.scale_by(kZoom); break;         // 放大
        case SDLK_X: if (shift) solid_.scale_by(1.0f / kZoom); break;  // 缩小
        default: return;
        }
        print_status();
    }

    // 把 CPU 帧缓冲画到屏幕：全项目唯一的 OpenGL 调用
    void draw_framebuffer() const {
        glBegin(GL_POINTS);
        for (int y = 0; y < kH; ++y) {
            for (int x = 0; x < kW; ++x) {
                const Vector3 c = fb_.pixel(x, y);
                glColor3f(c.x, c.y, c.z);
                // 像素坐标 → 数学坐标（原点在屏幕中心，y 向上）
                glVertex2f(x - kW / 2.0f + 0.5f, kH / 2.0f - y - 0.5f);
            }
        }
        glEnd();
    }

    void print_help() const {
        std::cout <<
            "控制（SDL 按键不区分大小写，用修饰键分工）：\n"
            "  相机  W/S 前后      A/D 左右      Q/E 摇头   R/F 俯仰\n"
            "  物体  Shift+W/S/A/D/N/M 移动   Shift+Z/X 缩放\n"
            "        Shift+Q/E/R/F/T/G 旋转(世界轴)   Ctrl+ 旋转(自身轴)\n"
            "  光源  I/K 上/下    J/L 左/右     U/O 靠近/远离\n"
            "  开关  B 背面剔除   H 消隐(z-buffer)   C 阴影贴图\n"
            "  裁剪  [ 缩小窗口   ] 放大窗口    ESC 退出\n";
    }

    // 状态打印走 linalg.h 里的 operator<<（Vector3/Quaternion 重载）
    void print_status() const {
        std::cout << "cam " << camera_.position
                  << " yaw " << camera_.yaw << " pitch " << camera_.pitch << "\n"
                  << "  pos " << solid_.position()
                  << "  rot " << solid_.orientation()
                  << "  scale " << solid_.scale()
                  << "  culling " << (solid_.culling_enabled() ? "on" : "off")
                  << "  zbuf " << (fb_.depth_test_enabled() ? "on" : "off")
                  << "  shadow " << (solid_.shadows_enabled() ? "on" : "off")
                  << "  light " << light_.position
                  << "  window " << int(clip_window_.scale() * 100.0f) << "%\n";
    }

    static constexpr int kW = 800;
    static constexpr int kH = 800;

    SDL_Window* window_ = nullptr;
    SDL_GLContext gl_ctx_ = nullptr;
    FrameBuffer fb_{kW, kH};
    SolidMesh solid_;
    PointLight light_;          // 点光源（Blinn-Phong），键盘可移动
    Camera camera_;             // 摄像机：WASD 移动、QE/RF 转视角
    ClipWindow clip_window_;    // Weiler–Atherton 裁剪窗口（屏幕中央矩形，可缩放）
};

int main() {
    App app;
    app.run();
    return 0;
}
