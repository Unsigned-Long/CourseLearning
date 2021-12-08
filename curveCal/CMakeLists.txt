cmake_minimum_required(VERSION 3.10)

project(curveCal VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)

include_directories(./include ./thirdparty/include)

link_directories(./thirdparty/libs)

aux_source_directory(./src SRC_DIR)

add_library(curveHandler SHARED ${SRC_DIR})

add_executable(curveCal ./main.cpp)

target_link_libraries(curveCal angle curveHandler)

