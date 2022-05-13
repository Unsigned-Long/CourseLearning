set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(OpenCV REQUIRED)

aux_source_directory(./src SRC_FILES)

add_executable(${CMAKE_PROJECT_NAME} main.cpp ${SRC_FILES})

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/thirdparty
  ./include
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
  ${OpenCV_LIBS}
  pthread
)