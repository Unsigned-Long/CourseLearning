cmake_minimum_required(VERSION 3.10)

project(optimization VERSION 1.0)

add_executable(${CMAKE_PROJECT_NAME}_gauss_newton ${CMAKE_SOURCE_DIR}/src/gauss_newton.cpp)

find_package(Ceres)
find_package(Eigen3)

include_directories(${EIGEN_INCLUDE_DIRS})

add_executable(${CMAKE_PROJECT_NAME}_use_ceres ${CMAKE_SOURCE_DIR}/src/use_ceres.cpp)

target_include_directories(${CMAKE_PROJECT_NAME}_use_ceres PRIVATE ${CERES_INCLUDE_DIRS})

target_link_libraries(${CMAKE_PROJECT_NAME}_use_ceres ${CERES_LIBRARIES})
