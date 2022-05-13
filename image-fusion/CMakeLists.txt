cmake_minimum_required(VERSION 3.10)

project(fusion VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE "Release")

add_subdirectory(${CMAKE_SOURCE_DIR}/src)
