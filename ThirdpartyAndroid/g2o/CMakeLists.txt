# g2o 下的 CMakeLists.txt
cmake_minimum_required(VERSION 3.18.1)
project(g2o)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -W")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -W")
configure_file(config.h.in ${g2o_SOURCE_DIR}/config.h)
include_directories(g2o/core g2o/solvers g2o/stuff g2o/types)
include_directories(../eigen-3.4.0)
file(GLOB SRC_LIST "g2o/types/*.cpp" "g2o/core/*.cpp" "g2o/stuff/*.cpp" 
"g2o/stuff/*.c")
add_library(g2o SHARED ${SRC_LIST})

