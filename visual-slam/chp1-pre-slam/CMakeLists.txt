# Sets the minimum required version of cmake
cmake_minimum_required(VERSION 3.10)

# Set the name of the project
project(preslam VERSION 0.0.1)

# Add include directories to the build
include_directories(${CMAKE_SOURCE_DIR}/include)

# Set a environment variable to the lib name
set(LIB_NAME helloslam)

# set the lib output directory
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

# set the bin output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# Add a library to the project using the 'helloslam.cpp'
add_library(${LIB_NAME} SHARED ${CMAKE_SOURCE_DIR}/src/helloslam.cpp)

# Add an executable to the project using the 'main.cpp'
add_executable(${CMAKE_PROJECT_NAME} ${CMAKE_SOURCE_DIR}/main.cpp)

# link the lib to the executable
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE  ${LIB_NAME})
