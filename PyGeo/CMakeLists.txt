cmake_minimum_required(VERSION 3.10)

project(pygeo VERSION 1.0)

SET(CMAKE_BUILD_TYPE "Release")

# SET(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_CXX_STANDARD 17)

find_package(PCL 1.3 REQUIRED io common visualization)

include_directories(./include ./thirdparty/include ${PCL_INCLUDE_DIRS})

aux_source_directory(./src SRC_DIR)

link_directories(./thirdparty/lib ${PCL_LIBRARY_DIRS})

add_executable(pygeo ./main.cpp ${SRC_DIR})

add_definitions(${PCL_DEFINITIONS})

target_link_libraries(pygeo ${PCL_LIBRARIES} colorPrj)
