# 用android studio交叉编译时对应的vins模块cmakelist.txt
#cmake_minimum_required(VERSION 2.8.3)
cmake_minimum_required(VERSION 3.18.1)
project(vins_no_ros_library)
add_definitions(-DANDROID_ON_) #表示开启安卓编译,会取消ros
#add_definitions(-DVIEWER_ON_)  #表示开启显示相关功能

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

if(CMAKE_BUILD_TYPE MATCHES "Debug")
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -O2 -g -fstack-protector -fsanitize=address -fno-omit-frame-pointer")
endif()

# 1. 基础配置
# 加上这两个宏，确保 boost 和 std::random_shuffle 能过
add_definitions(-DBOOST_NO_CXX98_FUNCTION_BASE)
add_definitions(-D_LIBCPP_ENABLE_CXX17_REMOVED_UNARY_BINARY_FUNCTION)
# 2. 定义当前目录变量 (关键！防止路径乱套)
set(CURRENT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(THIRDPARTY_DIR ${CURRENT_DIR}/ThirdpartyAndroid)


# 3. 准备依赖库 (头文件)
# --- Eigen ---
include_directories(${THIRDPARTY_DIR}/eigen-3.3.7)
include_directories(${THIRDPARTY_DIR}/eigen-3.3.7/Eigen)

# --- Sophus ---
include_directories(${THIRDPARTY_DIR}/Sophus)

# --- Boost ---
set(BOOST_ROOT "${THIRDPARTY_DIR}/boost-1_76_0")
include_directories(${BOOST_ROOT}/include)

# 4. 准备 OpenCV
# 建议这里使用相对路径或者由父级传入，但既然你写死路径了，先保留
set(OpenCV_DIR ${THIRDPARTY_DIR}/opencv-4.10.0-android-sdk/OpenCV-android-sdk/sdk/native/jni)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# --- Ceres Library (手动导入方式) ---
# 1. 设置Ceres根目录
set(CERES_ROOT ${THIRDPARTY_DIR}/ceres-solver-1.14.0)
# 3. 导入动态库libceres.so（SHARED对应.so文件）
add_library(ceres SHARED IMPORTED)
set_target_properties(ceres PROPERTIES
        IMPORTED_LOCATION ${CERES_ROOT}/lib/libceres.so  # 动态库路径
        # 可选：指定架构（适配高通arm64-v8a，增强交叉编译兼容性）
#        IMPORTED_ARCHITECTURES ARM64
)

# 3. 逐个导入依赖的Boost静态库模块
## 模块1: boost_system
add_library(boost_system STATIC IMPORTED)
set_target_properties(boost_system PROPERTIES
        IMPORTED_LOCATION "${BOOST_ROOT}/lib/libboost_system-clang-mt-a64-1_76.a"
)

## 模块3: boost_filesystem
add_library(boost_filesystem STATIC IMPORTED)
set_target_properties(boost_filesystem PROPERTIES
        IMPORTED_LOCATION "${BOOST_ROOT}/lib/libboost_filesystem-clang-mt-a64-1_76.a"
)

## 模块5: boost_program_options
add_library(boost_program_options STATIC IMPORTED)
set_target_properties(boost_program_options PROPERTIES
        IMPORTED_LOCATION "${BOOST_ROOT}/lib/libboost_program_options-clang-mt-a64-1_76.a"
)

set(BASE_DIR_camera_models ${CURRENT_DIR}/camera_models)
set(BASE_DIR_vins_estimator ${CURRENT_DIR}/vins_estimator)

# ---编译Library camera_models ---
add_library(camera_models STATIC
    ${BASE_DIR_camera_models}/src/chessboard/Chessboard.cc
    ${BASE_DIR_camera_models}/src/calib/CameraCalibration.cc
    ${BASE_DIR_camera_models}/src/camera_models/Camera.cc
    ${BASE_DIR_camera_models}/src/camera_models/CameraFactory.cc
    ${BASE_DIR_camera_models}/src/camera_models/CostFunctionFactory.cc
    ${BASE_DIR_camera_models}/src/camera_models/PinholeCamera.cc
    ${BASE_DIR_camera_models}/src/camera_models/PinholeFullCamera.cc
    ${BASE_DIR_camera_models}/src/camera_models/CataCamera.cc
    ${BASE_DIR_camera_models}/src/camera_models/EquidistantCamera.cc
    ${BASE_DIR_camera_models}/src/camera_models/ScaramuzzaCamera.cc
    ${BASE_DIR_camera_models}/src/sparse_graph/Transform.cc
    ${BASE_DIR_camera_models}/src/gpl/gpl.cc
    ${BASE_DIR_camera_models}/src/gpl/EigenQuaternionParameterization.cc)


# 9. 设置头文件导出 (PUBLIC)
# 【核心逻辑】这里用了 PUBLIC，意味着谁链接了 camera_models，谁就能自动看到下面这些头文件目录
target_include_directories(camera_models PUBLIC
        ${BASE_DIR_camera_models}/include
        ${THIRDPARTY_DIR}/Sophus
        ${THIRDPARTY_DIR}/eigen-3.3.7
        ${BOOST_ROOT}/include
        ${OpenCV_INCLUDE_DIRS}
        ${CERES_ROOT}/include
        ${CERES_ROOT}/include/ceres/internal/miniglog

)

target_link_libraries(camera_models PUBLIC
        boost_filesystem boost_program_options boost_system
        ${OpenCV_LIBS}
        ceres
        log
        android
)
message(WARNING "OpenCV_LIBS: ${OpenCV_LIBS}")


# ---编译Library vins_lib ---
file(GLOB HELPERS_SOURCEFILES "${BASE_DIR_vins_estimator}/Thirdparty/Helpers/*.cpp")
add_library(vins_lib STATIC
    ${BASE_DIR_vins_estimator}/src/estimator/parameters.cpp
    ${BASE_DIR_vins_estimator}/src/estimator/estimator.cpp
    ${BASE_DIR_vins_estimator}/src/estimator/feature_manager.cpp
    ${BASE_DIR_vins_estimator}/src/factor/pose_local_parameterization.cpp
    ${BASE_DIR_vins_estimator}/src/factor/projectionTwoFrameOneCamFactor.cpp
    ${BASE_DIR_vins_estimator}/src/factor/projectionTwoFrameTwoCamFactor.cpp
    ${BASE_DIR_vins_estimator}/src/factor/projectionOneFrameTwoCamFactor.cpp
    ${BASE_DIR_vins_estimator}/src/factor/marginalization_factor.cpp
    ${BASE_DIR_vins_estimator}/src/utility/utility.cpp
    # ${BASE_DIR_vins_estimator}/src/utility/visualization.cpp
    # ${BASE_DIR_vins_estimator}/src/utility/CameraPoseVisualization.cpp
    ${BASE_DIR_vins_estimator}/src/initial/solve_5pts.cpp
    ${BASE_DIR_vins_estimator}/src/initial/initial_aligment.cpp
    ${BASE_DIR_vins_estimator}/src/initial/initial_sfm.cpp
    ${BASE_DIR_vins_estimator}/src/initial/initial_ex_rotation.cpp
    ${BASE_DIR_vins_estimator}/src/featureTracker/feature_tracker.cpp
    ${HELPERS_SOURCEFILES})

# 9. 设置头文件导出 (PUBLIC)
# 【核心逻辑】这里用了 PUBLIC，意味着谁链接了 vins_lib，谁就能自动看到下面这些头文件目录
target_include_directories(vins_lib PUBLIC
        ${BASE_DIR_vins_estimator}/src
        ${BASE_DIR_vins_estimator}/Thirdparty/Helpers
)

target_link_libraries(vins_lib PUBLIC
        camera_models
        log
        android
)