cmake_minimum_required(VERSION 3.10)

project(visual_odometry)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV)

include_directories(${OpenCV_INCLUDE_DIRS})

link_libraries(${OpenCV_LIBS})

add_executable(${CMAKE_PROJECT_NAME}_orb ${CMAKE_SOURCE_DIR}/src/orb.cpp)