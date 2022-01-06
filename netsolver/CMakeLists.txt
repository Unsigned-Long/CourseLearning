cmake_minimum_required(VERSION 3.10)

project(netSolver VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3)

include_directories(./include ${EIGEN_INCLUDE_DIRS})

aux_source_directory(./src SRC_DIR)

add_executable(${PROJECT_NAME} ./main.cpp ${SRC_DIR})