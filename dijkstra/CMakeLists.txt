cmake_minimum_required(VERSION 3.10)

project(dijkstra VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)

include_directories(./include)

aux_source_directory(./src SRC_DIR)

add_executable(dijkstra main.cpp ${SRC_DIR})