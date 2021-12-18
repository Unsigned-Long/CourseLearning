cmake_minimum_required(VERSION 3.10)

project(convex VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)

set(LIBRARY_OUTPUT_PATH ../bin)

include_directories(./include ./thirdparty/include)

aux_source_directory(./src SRC_DIR)

add_library(conhull SHARED ${SRC_DIR})

add_executable(${PROJECT_NAME} ./main.cpp)

target_link_libraries(${PROJECT_NAME} conhull)