cmake_minimum_required(VERSION 3.10)

project(uwbindoor VERSION 1.0)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TYPE "Release")

find_package(Ceres)

find_package(RapidJSON)

find_package(Eigen3)

include_directories(./include ./thirdparty/include ${CERES_INCLUDE_DIRS} ${RAPIDJSON_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})

link_directories(./thirdparty/lib)

aux_source_directory(./src SRC_DIR)

add_executable(uwbindoor ./main.cpp ${SRC_DIR})

add_library(uwb SHARED ${SRC_DIR})

# add_library(uwb STATIC ${SRC_DIR})

target_link_libraries(uwbindoor ${CERES_LIBRARIES} ${RAPIDJSON_LIBRARIES} datetime)

target_link_libraries(uwb ${CERES_LIBRARIES} ${RAPIDJSON_LIBRARIES} datetime)
