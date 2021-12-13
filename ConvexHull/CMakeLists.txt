cmake_minimum_required(VERSION 3.10)

project(convex VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)

include_directories(./include ./thirdparty/include)

aux_source_directory(./src SRC_DIR)

add_executable(convex ./main.cpp ${SRC_DIR})