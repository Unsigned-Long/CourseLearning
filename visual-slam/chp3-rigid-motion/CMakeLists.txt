cmake_minimum_required(VERSION 3.10)

project(rigidmotion)

find_package(Eigen3)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

add_executable(${CMAKE_PROJECT_NAME}_eigen ${CMAKE_SOURCE_DIR}/src/part1-use-eigen/main.cpp)

add_executable(${CMAKE_PROJECT_NAME}_geometry ${CMAKE_SOURCE_DIR}/src/part2-use-geometry/main.cpp)

add_executable(${CMAKE_PROJECT_NAME}_case ${CMAKE_SOURCE_DIR}/src/part3-case/main.cpp)

target_include_directories(${CMAKE_PROJECT_NAME}_eigen PRIVATE ${EIGEN_INCLUDE_DIRS})

target_include_directories(${CMAKE_PROJECT_NAME}_geometry PRIVATE ${EIGEN_INCLUDE_DIRS})

target_include_directories(${CMAKE_PROJECT_NAME}_case PRIVATE ${EIGEN_INCLUDE_DIRS})