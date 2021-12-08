cmake_minimum_required(VERSION 3.10)

project(resVPHandler VERSION 1.0)

set(CMAKE_BUILD_TYPE "Release")

find_package(RapidJSON)

find_package(Eigen3)

find_package(Ceres REQUIRED)

include_directories(./include ${CERES_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${RAPIDJSON_INCLUDE_DIRS} ./thirdparty/include)

aux_source_directory(./src SRC_DIR)

add_executable(resVPHandler ./main.cpp ${SRC_DIR})

target_link_libraries(resVPHandler ${CERES_LIBRARIES})