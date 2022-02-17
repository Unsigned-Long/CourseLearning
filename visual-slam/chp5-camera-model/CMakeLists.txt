cmake_minimum_required(VERSION 3.10)

project(cameramodel VERSION 1.0)

find_package(OpenCV)
find_package(Sophus)
find_package(Eigen3)
find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization)

set (CMAKE_DISABLE_FIND_PACKAGE_fmt ON)

include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE "Release")

add_executable(${CMAKE_PROJECT_NAME}_useopencv ${CMAKE_SOURCE_DIR}/src/useopencv.cpp)

target_link_libraries(${CMAKE_PROJECT_NAME}_useopencv PRIVATE ${OpenCV_LIBS})

add_executable(${CMAKE_PROJECT_NAME}_undistorted ${CMAKE_SOURCE_DIR}/src/undistorted.cpp)

target_link_libraries(${CMAKE_PROJECT_NAME}_undistorted PRIVATE ${OpenCV_LIBS})


add_executable(${CMAKE_PROJECT_NAME}_depthPointCloud ${CMAKE_SOURCE_DIR}/src/depthPointCloud.cpp)

target_link_libraries(${CMAKE_PROJECT_NAME}_depthPointCloud PRIVATE 
                      ${OpenCV_LIBS}
                      ${PCL_LIBRARIES})

 add_executable(${CMAKE_PROJECT_NAME}_rgbd ${CMAKE_SOURCE_DIR}/src/rgbd.cpp)

target_include_directories(${CMAKE_PROJECT_NAME}_rgbd PRIVATE ${SOPHUS_INCLUDE_DIRS})

target_link_libraries(${CMAKE_PROJECT_NAME}_rgbd PRIVATE 
                      ${OpenCV_LIBS}
                      ${PCL_LIBRARIES}
                      Sophus::Sophus)

