# 小型 CPU 渲染引擎

纯 CPU 软件渲染器：**所有渲染计算在 CPU 上完成，OpenGL 仅使用 `GL_POINTS` 绘制像素，不调用任何其他渲染 API**。

技术栈：**C++17**（面向对象 + RAII）+ **SDL3**（窗口层）+ **OpenGL**（仅画点）+ **MinGW GCC 14.2 / Ninja**（64 位构建），在 VS Code 中开发。

> 交作业说明：SDL3 在本项目里只承担 GLUT 的角色（开窗口、收键盘、交换缓冲），渲染与"纯 CPU + OpenGL 画点"的要求完全一致。如课程明确要求 GLUT，提交前与老师说明即可。

## 当前阶段：OBJ/MTL 导入 + 纹理映射 + 平滑着色 + z-buffer 消隐 + Blinn-Phong + 阴影贴图

模型**在启动时从 `src/ball.obj` 导入**（asset/ 路径兼容保留；加载器：`src/objloader.h` +
`src/mtlloader.h`）。这份 OBJ 里有两个对象：悬浮的"球体"（960 三角面）和一块扁平的
"立方体"台板——装配规则是**水平足迹最大且扁平的对象作为固定地面**（承接阴影、不做模型
变换），其余对象合成可变换模型，等比归一化到最大边长 2、底部悬停在地面（与内置球同款
悬浮间隙）；MTL 的 Kd/Ks/Ns 接入 Blinn-Phong。OBJ 打不开或解析失败时回退到
**内置悬浮球 + 30×30 地面**，无模型文件也能跑。

**纹理映射（已实现，透视校正）**：MTL 的 `map_Kd` 经 stb_image 读入像素，UV 作为顶点
属性穿过整条管线——插值的是屏幕仿射量 `(u/w, v/w)` 与 `1/w`，像素处相除还原 u/v，
从根上避免旧项目"贴图会游动"的 bug；采样为双线性 + repeat 回绕 + clamp-to-edge。
**平滑着色**：OBJ 顶点法线（vn）同样逐像素插值后再进 Blinn-Phong——vn 数据平（平直
着色导出）就是平面着色，数据滑（Shade Smooth 导出）就是逐像素平滑，外观由模型数据决定。
每帧的渲染流程（全部在 CPU 上）：

0. **资源导入**：启动时 `SolidMesh::build_scene` 调 `load_obj("src/ball.obj")` 装配场景，并用 stb_image 读入 `map_Kd` 纹理（kGeom 静态初始化，一次）；
1. **阴影贴图 pass**：从光源视角把模型全部面渲染进一张 512² 深度图（复用 AET 填充，只写深度，C 键开关）；
2. 由键盘状态组合**模型矩阵**（平移 · **四元数旋转** · 缩放，只作用于小球）；
2. 16 个顶点经 **model → view → projection** 变换到裁剪空间；
4. **背面剔除**（B 键开关）：面法线 · 视线 ≤ 0 的面直接跳过，只画朝向相机的面；
5. **视锥体裁剪**：近平面 + NDC 保护带（Sutherland–Hodgman 半空间裁剪 ×5，防止相机身后的面产生天文坐标）；
6. **齐次除法** → NDC → **视口变换**（y 翻转），NDC 深度一并输出（近 0 远 1）；
7. **Weiler–Atherton 算法**把面多边形裁剪进可缩放的矩形窗口（`[` / `]` 调整大小）；
8. **AET 扫描线算法**填充面内部，逐像素执行**片元着色器**：由"屏幕坐标 + NDC 深度"精确重建片元世界坐标 → 光源空间采样阴影图（阴影中只剩环境光）→ **Blinn-Phong**（(N·L) 漫反射 + (N·H)^s 高光，光源 I/K/J/L/U/O 移动，光源处的黄色小球即灯泡，十字标记精确位置），逐像素过 **z-buffer 深度测试**（H 键开关）；
9. **DDA 算法**描出多边形轮廓（识别缓冲 id 门槛），写入帧缓冲；
10. OpenGL 用 `GL_POINTS` 把帧缓冲画上屏（全项目唯一 GL 调用）。

键盘控制（按住可连续触发；SDL 按键不区分大小写，用修饰键分工——
**无修饰键 = 摄像机，Shift = 小球变换，Ctrl = 小球自身轴旋转**）：

| 按键 | 功能 |
|---|---|
| W / S / A / D | 摄像机 前后 / 左右移动 |
| Q / E | 摄像机 左右摇头 |
| R / F | 摄像机 抬头 / 低头 |
| Shift + W/S/A/D/N/M | 小球平移（上/下/左/右/靠近/远离） |
| Shift + Q/E/R/F/T/G | 小球旋转（世界轴） |
| Ctrl + Q/E/R/F/T/G | 小球旋转（自身轴） |
| Shift + Z / X | 小球 放大 / 缩小 |
| I / K / J / L / U / O | 点光源 下/上/左/右/靠近/远离 |
| B / H / C | 背面剔除 / z-buffer 消隐 / 阴影贴图 开关 |
| ] / [ | 放大 / 缩小裁剪窗口 |
| ESC | 退出 |

> 摄像机与物体的分工：姿态（小球）用**四元数**存储（无万向节锁），世界轴/自身轴旋转是同一姿态的左乘/右乘（即 Unity 的 `Space.World` / `Space.Self`）；摄像机为 FPS 式位置 + 偏航/俯仰。
> 完整的键盘 / 鼠标交互说明（含步进参数、与旧大作业键位对照、后续规划）见 **[CONTROLS.md](CONTROLS.md)**。

> 上一阶段已实现三角形光栅化 + z-buffer + Blinn-Phong，代码归档在 `src/rasterizer_backup.h`；后续按学习路线以更完整的形式恢复（透视校正插值、Sutherland-Hodgman 视锥裁剪、阴影贴图等）。

## 目录结构

```
softrender/
├─ CMakeLists.txt            构建脚本（MinGW + Ninja）
├─ CONTROLS.md               键盘 / 鼠标交互说明（按键表、键位对照、后续规划）
├─ asset/                    （已迁出：模型资产现放在 src/ 下）
├─ third_party/SDL3/         SDL3 开发包（include / lib / bin，自包含）
├─ src/
│  ├─ linalg.h               向量/齐次坐标/4×4 矩阵/**四元数**，运算符重载（a+b、a·b、M*v）
│  ├─ mtlloader.h            MTL 加载器：Kd/Ks/Ns/d/map_Kd（纹理路径回退解析）
│  ├─ texture.h              纹理：stb_image 读图 + 双线性采样（v 轴翻转 / 回绕 / 边缘钳制）
│  ├─ stb_image.h            第三方单头图像库 v2.30（public domain）
│  ├─ objloader.h            OBJ 加载器：v/vt/vn/f/mtllib/usemtl/o/g/s，扇形三角化兜底
│  ├─ soft_renderer.h        FrameBuffer（AET 填充 + z-buffer）+ ClipWindow + Weiler–Atherton
│  │                         裁剪 + SolidMesh（OBJ 导入装配 + 内置回退场景）
│  ├─ rasterizer_backup.h    上一阶段归档：三角形光栅化 + Blinn-Phong + z-buffer
│  ├─ ball.obj / ball.mtl / tietu.png    模型资产
│  └─ main.cpp               App 类（RAII）：SDL3 窗口 + 键盘交互 + GL_POINTS 上屏
├─ tests/                    回归测试（手动编译运行，见各文件头注释）+ 无头帧导出工具
└─ .vscode/                  VS Code 任务 / 调试 / CMake 配置
```

## 在 VS Code 中使用

1. 用 VS Code 打开本文件夹（`文件 → 打开文件夹`）。
2. 终端 → 运行任务 → **首次配置（MinGW + Ninja）**（只需一次）。
3. `Ctrl+Shift+B` 构建（默认任务"构建"）。
4. 终端 → 运行任务 → **运行**；或按 F5 调试（需要 C/C++ 扩展，走 MinGW gdb）。

命令行等价操作：

```bash
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_COMPILER=D:/mingw64/bin/g++.exe \
      -DCMAKE_MAKE_PROGRAM=D:/mingw64/bin/ninja.exe
cmake --build build
./build/renderer.exe
```

> 前置条件：
> MinGW 位于无括号路径 `D:\mingw64`（目录联接，指向 `D:\Software(English)\Mingw14.2.0\mingw64`）。若联接丢失，重建：
> `powershell -Command "New-Item -ItemType Junction -Path 'D:\mingw64' -Target 'D:\Software(English)\Mingw14.2.0\mingw64'"`
>
> 注：项目目录名已改为纯英文 `softrender`，gdb 可直接在 `D:\Code\softrender` 下工作，
> 不再需要旧的 `D:\softrender` 目录联接。

## 渲染管线 ↔ 代码对照

| 管线阶段 | 位置 |
|---|---|
| OBJ/MTL 导入（模型来源） | `objloader.h` / `mtlloader.h`；装配在 `SolidMesh::build_scene`（kGeom 静态初始化时执行） |
| 法线变换（模型空间 → 世界） | `linalg.h`：`transform_vector`（不带平移；均匀缩放靠归一化抵消） |
| 平滑着色（vn 逐像素插值） | 视锥裁剪 `clip_polygon_frustum_n` → W-A `weiler_atherton`（PolyVert 携带法线）→ AET `fill_polygon` 逐像素插值 → shader 里 normalize |
| UV 透视校正插值 | 顶点属性 `(u/w, v/w)` 与 `1/w` 屏幕仿射插值（`PolyVert`/AET 边表），像素处相除还原——`texture.h` 双线性采样 |
| 纹理采样（map_Kd） | `texture.h`：`load_texture`（stb_image）+ `sample_texture`（双线性 / v 翻转 / 回绕 / 边缘钳制）；shader 内接入 Blinn-Phong 的 albedo |
| 模型矩阵组合（平移·四元数旋转·缩放） | `SolidMesh::model_matrix`，旋转矩阵来自 `Quaternion::to_matrix` |
| 视图/投影矩阵 | `linalg.h`：`look_at` / `perspective` |
| 顶点变换 → 裁剪空间 | `linalg.h`：`transform_point_clip` |
| 背面剔除（面法线 · 视线，B 键开关） | `SolidMesh::draw` 中 `normal.dot(eye - w0)` |
| 近平面裁剪（3D，单平面 Sutherland–Hodgman） | `soft_renderer.h`：`clip_polygon_near` |
| 齐次除法 → NDC | `SolidMesh::draw` 中 `inv_w = 1/w` |
| 视口变换（y 翻转） | `project_to_screen` |
| 矩形窗口多边形裁剪（Weiler–Atherton） | `soft_renderer.h`：`weiler_atherton` |
| 区域填充（AET 扫描线：GET + 活性边表） | `soft_renderer.h`：`FrameBuffer::fill_polygon` |
| 消隐（z-buffer 深度测试，H 键开关） | `FrameBuffer::plot`(深度重载)；深度在边/span 间线性插值 |
| 着色（Blinn-Phong，逐像素） | `soft_renderer.h`：`blinn_phong` + `PointLight`（位置键盘可移动），经 `fill_polygon` 的 PixelShader 接入 |
| 阴影（阴影贴图，C 键开关） | `render_depth`（光源深度 pass）+ 片元着色器内光源空间采样 + bias 比较 |
| 描边消隐（识别缓冲 id 门槛） | `FrameBuffer::draw_line`：描边只画在自己面的填充赢下的像素上 |
| DDA 线光栅化 | `soft_renderer.h`：`FrameBuffer::draw_line` |
| 上屏 | `main.cpp`：`App::draw_framebuffer`（全项目唯一的 OpenGL 调用） |

## 构建要点（踩坑记录）

| 问题 | 原因 | 解决 |
|---|---|---|
| MinGW 构建时 `'D:\Software' 不是命令` | Ninja 经 `cmd /C "cd . && 编译器 && cd ."` 链接，括号路径截断命令 | 建无括号目录联接 `D:\mingw64`，配置时显式指定编译器路径 |
| SDL3 链接报 `cannot find -lSDL3` | GNU ld 打不开含中文的绝对路径（-L 搜索/显式路径均失败） | 把 libSDL3.dll.a 复制进 build 目录，用 `-L.` 相对路径搜索 |
| F5 报 `No such file or directory` | gdb 无法处理含中文的工作目录（编码不匹配） | 项目目录已改纯英文名 `softrender`，问题消失 |
| `<cmath>` 全线报错 | 自定义头文件命名为 `math.h` 会遮蔽系统 `<math.h>` | 命名为 `linalg.h`，避开保留名 |
| 修改 CMakeLists 后构建没变化 | 本机 cmake 路径含括号，禁用了自动重生成 | 手动重新配置（"首次配置"任务），或删掉 build 目录重来 |
| 换目录后构建报 `CMakeCache.txt directory ... is different` | build 目录缓存了旧绝对路径 | 删掉 build 目录，重新"首次配置" |
| OBJ 集成后 Release 版启动即段错误（-O0 编译的测试又全通过） | Git 自带旧 `libstdc++-6.dll`，PATH 里抢占加载；GCC 14 内联进 exe 的 ifstream/locale 代码与它不匹配 | CMakeLists 把编译器自带的三份运行库（libstdc++/libgcc/libwinpthread）复制进 build 目录——Windows 加载 DLL 先查 exe 所在目录；改 CMakeLists 后需手动重新配置 |

## 从旧项目（图形学大作业）迁移清单

- [x] OBJ/MTL 加载（`objloader.h` / `mtlloader.h`：v/vt/vn/f、usemtl 分组、Kd/Ks/Ns、map_Kd 路径解析）
- [x] 完整纹理映射（`texture.h`：stb_image 读图 + UV **透视校正插值** `(u/w,1/w)` + 双线性采样——旧版"贴图游动"bug 的根治版）
- [ ] 完整纹理映射：stb_image 读图 + UV **透视校正插值**（旧版贴图会"游动"的 bug；map_Kd 已解析到 asset/tietu.png 待接入）
- [ ] 阴影贴图 pass（光源视角深度图 + bias 比较）
- [ ] 视锥体裁剪其余 5 个面（近平面已用单平面 Sutherland–Hodgman 实现；屏幕窗口裁剪已用 Weiler–Atherton 实现）
- [x] z-buffer 消隐（已在新架构恢复：NDC 深度 + 逐像素测试，H 键开关）
- [x] 阴影贴图 pass（光源视角 512² 深度图 + 逐像素 bias 比较，C 键开关）
- [ ] 恢复三角形光栅化（当前为任意多边形 AET 填充；三角形 + 重心坐标版待 OBJ 模型阶段引入）
- [ ] 光线追踪分支（BVH + Möller–Trumbore + Whitted 递归）
- [ ] 修复旧版 `calculate_lighting` 缺 `return` 的 bug
- [ ] 相机漫游（WASD + 累积旋转矩阵，pitch 钳制 ±89°）

## 未来：CPU 渲染 → GPU 渲染的升级路径

窗口层（SDL3）不变，逐步替换渲染层：

1. **着色器化**：把 `soft_renderer.h` 的变换/光栅化/光照搬进 vertex/fragment shader（GL 4.5 兼容 profile 起步）；
2. **现代 OpenGL**：VAO/VBO + 自定义 shader，帧缓冲里的像素变成 GPU 上的 draw call；
3. **可选 SDL_GPU**：SDL3 内置跨 Vulkan/D3D12/Metal 的 GPU 抽象，不写裸 Vulkan 也能体验现代管线思想。
