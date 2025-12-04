# 提供给android studio cpp/目录下的固定cmakelist.txt
# 1. 基础版本设定
cmake_minimum_required(VERSION 3.10.2)
project(vins_root)
add_definitions(-DANDROID_ON_) #表示开启安卓编译,会取消ros
# =======================================================
#  根据构建类型动态控制
# =======================================================

# 打印一下当前是什么模式 (Debug 还是 Release)
message(STATUS ">>>>>> Current Build Type: ${CMAKE_BUILD_TYPE} <<<<<<")

if("${CMAKE_BUILD_TYPE}" STREQUAL "Release")

    message(STATUS "🔥 Release Mode Detected: Enabling Aggressive Optimizations!")

    # 1. 基础优化
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

    # 2. 数学与循环
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math -funroll-loops -fno-strict-aliasing")

    # 3. LTO (注意：如果编译报错，可以注释掉这部分)
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -flto")
    #set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -flto")
    #set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -flto")

else()

    message(STATUS "🐞 Debug Mode Detected: Optimizations Disabled for Debugging.")
    # Debug 模式下通常默认是 -O0 -g，不需要额外配置

endif()

# 2. 编译参数
set(CMAKE_VERBOSE_MAKEFILE on)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++17")

# -------------------------------------------------------
# 配置 OpenCV (通用配置，x86和arm64都需要)
# -------------------------------------------------------
# 注意：确保路径正确
set(OpenCV_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/VINS-Fusion/ThirdpartyAndroid/opencv-4.10.0-android-sdk/OpenCV-android-sdk/sdk/native/jni/include")
include_directories(${OpenCV_INCLUDE_DIR})

# 动态引入 OpenCV
add_library(lib_opencv SHARED IMPORTED)

# 自动根据 ABI (x86_64 或 arm64-v8a) 定位 jniLibs 下的库
set_target_properties(lib_opencv PROPERTIES IMPORTED_LOCATION
        "${CMAKE_CURRENT_SOURCE_DIR}/VINS-Fusion/ThirdpartyAndroid/opencv-4.10.0-android-sdk/OpenCV-android-sdk/sdk/native/libs/${ANDROID_ABI}/libopencv_java4.so")

# -------------------------------------------------------
# 目标 A: 生成 APK 用的 JNI 库 (native-lib)
# 这个目标在模拟器和真机上都会编译
# -------------------------------------------------------
add_library(native-lib SHARED native-lib.cpp)

# 强制 4KB 对齐
target_link_options(native-lib PRIVATE "-Wl,-z,max-page-size=4096")

# 链接库 (只链接基础库，不依赖 vins，方便模拟器调试 UI)
target_link_libraries(native-lib
        lib_opencv
        log
        android
        -ljnigraphics
)

# =======================================================
#    核心隔离区：只有 ARM64 (真机) 才编译 ORB-SLAM3
# =======================================================
if(${ANDROID_ABI} STREQUAL "arm64-v8a")

    # 1. 注入解决 Boost 兼容性的宏 (仅限编译算法时使用)
    add_definitions(-DBOOST_NO_CXX98_FUNCTION_BASE)
    add_definitions(-D_LIBCPP_ENABLE_CXX17_REMOVED_UNARY_BINARY_FUNCTION)

    # 2. 编译子模块 (YsSlam 库)
    add_subdirectory(VINS-Fusion)

    # -------------------------------------------------------
    # 目标 B: 生成可执行文件 (双目+IMU) - 仅限真机使用
    # -------------------------------------------------------
    # 确保 .cc 文件就在当前目录下
    add_executable(VINS_run vslamNodeTest3NoRos.cpp)

    # 强制 4KB 对齐 (XR2 必需)
    target_link_options(VINS_run PRIVATE "-Wl,-z,max-page-size=4096")

    # 链接库
    # orbslam3_lib 来自子目录生成
    target_link_libraries(VINS_run
            vins_lib
            log
            android
    )

endif()
