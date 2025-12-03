cmake_minimum_required(VERSION 2.8.3)
project(vins_no_ros)
add_definitions(-DANDROID_ON_) #表示开启安卓编译,会取消ros
add_definitions(-DVIEWER_ON_)  #表示开启显示相关功能

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

if(CMAKE_BUILD_TYPE MATCHES "Debug")
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -O2 -g -fstack-protector -fsanitize=address -fno-omit-frame-pointer")
endif()

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
include_directories(${Boost_INCLUDE_DIRS})

# find_package(OpenCV 4.2.0 REQUIRED)
find_package(OpenCV 4.10.0 REQUIRED)
message(WARNING "camera_models, OpenCV_VERSION: ${OpenCV_VERSION}")

# set(EIGEN_INCLUDE_DIR "/usr/local/include/eigen3")
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

set(BASE_DIR_camera_models ${PROJECT_SOURCE_DIR}/../camera_models)
set(BASE_DIR_vins_estimator ${PROJECT_SOURCE_DIR}/../vins_estimator)

set(CMAKE_MODULE_PATH ${BASE_DIR_vins_estimator}/cmake)
find_package(Eigen3)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${BASE_DIR_vins_estimator}/Thirdparty/Sophus
  ${BASE_DIR_vins_estimator}/Thirdparty/Helpers
)
include_directories(${BASE_DIR_camera_models}/include)
include_directories(${BASE_DIR_vins_estimator}/src)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bins)

add_executable(Calibrations 
    ${BASE_DIR_camera_models}/src/intrinsic_calib.cc
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

add_library(camera_models
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
# 查找 OpenMP 库
find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
    # 添加 OpenMP 编译标志（如 -fopenmp）
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

target_link_libraries(Calibrations ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} OpenMP::OpenMP_CXX)
target_link_libraries(camera_models ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
message(WARNING "OpenCV_LIBS: ${OpenCV_LIBS}")



file(GLOB HELPERS_SOURCEFILES "${BASE_DIR_vins_estimator}/Thirdparty/Helpers/*.cpp")

add_library(vins_lib
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
    ${BASE_DIR_vins_estimator}/src/featureTracker/feature_tracker.cpp)

target_link_libraries(vins_lib ${OpenCV_LIBS} ${CERES_LIBRARIES} OpenMP::OpenMP_CXX camera_models)

add_executable(vslam_test3_noros ${BASE_DIR_vins_estimator}/src/vslamNodeTest3NoRos.cpp ${HELPERS_SOURCEFILES})
target_link_libraries(vslam_test3_noros vins_lib)