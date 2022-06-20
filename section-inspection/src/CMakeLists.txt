set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

include_directories(include ${CMAKE_SOURCE_DIR}/thirdparty)

aux_source_directory(src SRC_FILES)

add_executable(${CMAKE_PROJECT_NAME} ./main.cpp ${SRC_FILES})
