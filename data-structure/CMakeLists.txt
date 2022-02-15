cmake_minimum_required(VERSION 3.10)

project(datastruct VERSION 0.0.1)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(${CMAKE_PROJECT_NAME} ${CMAKE_SOURCE_DIR}/main.cpp)