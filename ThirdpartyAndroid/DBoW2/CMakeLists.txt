# DBoW2 下的 CMakeLists.txt
cmake_minimum_required(VERSION 3.18.1)
project(DBoW2)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../boost-1_76_0/include)
include_directories(DBoW2 DUtils)
file(GLOB SRC_LIST "DBoW2/*.cpp" "DUtils/*.cpp")
include_directories(${OpenCV_INCLUDE_DIRS})
add_library(DBoW2 SHARED ${SRC_LIST})
target_link_libraries(DBoW2 ${OpenCV_LIBS})

