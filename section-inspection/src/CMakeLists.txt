set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(Eigen3 REQUIRED)

find_package(Sophus REQUIRED)

find_package(PCL REQUIRED)

include_directories(
  include
  ${CMAKE_SOURCE_DIR}/thirdparty
  ${Sophus_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

aux_source_directory(src SRC_FILES)

add_executable(${CMAKE_PROJECT_NAME} ./main.cpp ${SRC_FILES})

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE Sophus::Sophus ${PCL_LIBRARIES})
