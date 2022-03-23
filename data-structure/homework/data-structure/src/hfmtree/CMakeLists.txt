include_directories(.)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

add_executable(${CMAKE_PROJECT_NAME}_hfmtree main.cpp src/hfmtree.cpp)