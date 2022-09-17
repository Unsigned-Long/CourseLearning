set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(Eigen3)
find_package(RapidJSON)
find_package(Boost)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRC_FILES)

add_library(${CMAKE_PROJECT_NAME} SHARED ${SRC_FILES})

target_include_directories(
        ${CMAKE_PROJECT_NAME} PUBLIC
        ${EIGEN_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${RAPIDJSON_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

target_link_libraries(
        ${CMAKE_PROJECT_NAME} PUBLIC
        ${Boost_LIBRARIES}
)

add_executable(${CMAKE_PROJECT_NAME}_prog ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

target_link_libraries(
        ${CMAKE_PROJECT_NAME}_prog PRIVATE
        ${CMAKE_PROJECT_NAME}
)