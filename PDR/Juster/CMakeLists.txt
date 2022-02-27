# header
cmake_minimum_required(VERSION 3.10)

project(juster VERSION 1.0)

# settings
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_STANDARD_REQUIRED true)

set(CMAKE_BUILD_TYPE "Debug")

# set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# add exe
add_executable(${CMAKE_PROJECT_NAME}_hma ${CMAKE_SOURCE_DIR}/src/hma.cpp)

# add_executable(${CMAKE_PROJECT_NAME}_kalman ${CMAKE_SOURCE_DIR}/src/kalman.cpp)

add_executable(${CMAKE_PROJECT_NAME}_akalman ${CMAKE_SOURCE_DIR}/src/akalman.cpp)

# add_executable(${CMAKE_PROJECT_NAME}_ekalman ${CMAKE_SOURCE_DIR}/src/ekalman.cpp)

add_executable(${CMAKE_PROJECT_NAME}_signal ${CMAKE_SOURCE_DIR}/src/signal.cpp)

