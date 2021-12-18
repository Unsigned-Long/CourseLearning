cmake_minimum_required(VERSION 3.10)

project(tinCreator VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)

set(LIBRARY_OUTPUT_PATH ../bin)

include_directories(./include ./thirdparty/include/conhull ./thirdparty/include/geometry)

link_directories(./thirdparty/lib)

aux_source_directory(./src SRC_DIR)

add_library(tin SHARED ${SRC_DIR})

add_executable(${PROJECT_NAME} ./main.cpp)

target_link_libraries(tin conhull)

target_link_libraries(${PROJECT_NAME} tin)
