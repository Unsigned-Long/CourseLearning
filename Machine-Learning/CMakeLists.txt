cmake_minimum_required(VERSION 3.10)

project(machineLearning VERSION 1.0)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3)

find_package(Ceres)

include_directories(${EIGEN_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

include_directories(./sample/include ./regression/include ./statistics/include)

include_directories(./clustering/include ./bayes/include ./helper/include)

add_executable(machineLearning ./main.cpp)

target_link_libraries(machineLearning ${CERES_LIBRARIES})
