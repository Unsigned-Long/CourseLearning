cmake_minimum_required(VERSION 3.10)

project(optimization VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3)
include_directories(${EIGEN_INCLUDE_DIRS})

# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# g2o
find_package(g2o)
include_directories(${G2O_INCLUDE_DIRS})

# add_executable(${CMAKE_PROJECT_NAME}_gauss_newton ${CMAKE_SOURCE_DIR}/src/gauss_newton.cpp)

# add_executable(${CMAKE_PROJECT_NAME}_use_ceres ${CMAKE_SOURCE_DIR}/src/use_ceres.cpp)

# target_link_libraries(${CMAKE_PROJECT_NAME}_use_ceres ${CERES_LIBRARIES})

add_executable(${CMAKE_PROJECT_NAME}_use_g2o ${CMAKE_SOURCE_DIR}/src/use_g2o.cpp)

target_link_directories(${CMAKE_PROJECT_NAME}_use_g2o PUBLIC ${EIGEN_INCLUDE_DIRS})

target_link_libraries(${CMAKE_PROJECT_NAME}_use_g2o PUBLIC ${G2O_LIBRARIES})

