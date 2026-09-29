cmake_minimum_required(VERSION 3.16)
project(soft_renderer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 构建期不做重生成检查（规避本机 cmake.exe 路径含括号在 cmd 下的问题）。
# 修改本文件后需手动重新配置（VS Code: 终端 → 运行任务 → 首次配置）。
set(CMAKE_SUPPRESS_REGENERATION ON)

set(SDL3_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/SDL3)

if(NOT EXISTS ${SDL3_DIR}/include/SDL3/SDL.h)
    message(FATAL_ERROR
        "缺少 third_party/SDL3 —— 下载 SDL3-devel-*-mingw.tar.gz，"
        "把其中 x86_64-w64-mingw32 文件夹的内容复制到 third_party/SDL3/")
endif()

add_executable(renderer src/main.cpp)
target_include_directories(renderer PRIVATE src ${SDL3_DIR}/include)

# GNU ld 无法打开含中文的绝对路径（本工程路径含中文）：
# 把导入库复制进 build 目录，用 ASCII 相对路径 -L. 搜索链接
file(COPY ${SDL3_DIR}/lib/libSDL3.dll.a DESTINATION ${CMAKE_BINARY_DIR})
target_link_options(renderer PRIVATE -L.)
target_link_libraries(renderer PRIVATE -lSDL3 opengl32)

# SDL3.dll 复制到输出目录，保证运行时可加载
file(COPY ${SDL3_DIR}/bin/SDL3.dll DESTINATION ${CMAKE_BINARY_DIR})

# 编译器自带的 C++ 运行库复制到输出目录。Windows 加载 DLL 时先查 exe 所在
# 目录——若不这样做，运行时会从 PATH 抓到不匹配的副本（实测：Git 自带的
# 旧 libstdc++-6.dll 抢占加载，Release 构建在 ifstream/locale 处直接段错误，
# 而 Debug/-O0 恰好能跑）。修改本文件后需手动重新配置（见文件头说明）。
get_filename_component(MINGW_BIN "${CMAKE_CXX_COMPILER}" DIRECTORY)
foreach(dll libstdc++-6.dll libgcc_s_seh-1.dll libwinpthread-1.dll)
    if(EXISTS "${MINGW_BIN}/${dll}")
        file(COPY "${MINGW_BIN}/${dll}" DESTINATION ${CMAKE_BINARY_DIR})
    endif()
endforeach()
