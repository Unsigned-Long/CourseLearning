cmake_minimum_required(VERSION 3.10)

project(optimization VERSION 0.1)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_STANDARD_REQUIRED true)

# set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_BUILD_TYPE "Debug")

# algorithms

# add_executable(${CMAKE_PROJECT_NAME}_dichSearch ${CMAKE_SOURCE_DIR}/src/dichSearch.cpp)

# add_executable(${CMAKE_PROJECT_NAME}_eqinterSearch ${CMAKE_SOURCE_DIR}/src/eqinterSearch.cpp)

add_executable(${CMAKE_PROJECT_NAME}_fibonSearch ${CMAKE_SOURCE_DIR}/src/fibonSearch.cpp)
