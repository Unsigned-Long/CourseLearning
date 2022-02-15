cmake_minimum_required(VERSION 3.10)

project(leegroup VERSION 1.0)

find_package(Sophus)
find_package(Eigen3)

set (CMAKE_DISABLE_FIND_PACKAGE_fmt ON)

add_executable(${CMAKE_PROJECT_NAME} ${CMAKE_SOURCE_DIR}/src/leegroup.cpp)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${SOPHUS_INCLUDE_DIRS})

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${EIGEN3_INCLUDE_DIRS})

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE Sophus::Sophus)