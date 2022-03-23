cmake_minimum_required(VERSION 3.10)

project(ds LANGUAGES CXX VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(CMAKE_BUILD_TYPE "Debug")

add_subdirectory(${CMAKE_SOURCE_DIR}/src/linklist)

add_subdirectory(${CMAKE_SOURCE_DIR}/src/cirqueue)

add_subdirectory(${CMAKE_SOURCE_DIR}/src/hfmtree)

add_subdirectory(${CMAKE_SOURCE_DIR}/src/graph)