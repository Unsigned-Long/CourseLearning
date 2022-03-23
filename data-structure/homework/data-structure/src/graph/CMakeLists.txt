include_directories(.)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

add_executable(${CMAKE_PROJECT_NAME}_graph main.cpp src/graph.cpp)