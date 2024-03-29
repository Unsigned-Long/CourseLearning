cmake_minimum_required(VERSION 3.5)

project(QSection VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets DataVisualization)

# add resource file
file(GLOB_RECURSE QRC_SOURCE_FILES ${CMAKE_SOURCE_DIR}/resource.qrc)

qt6_add_resources(QRC_FILES ${QRC_SOURCE_FILES})

source_group("Resource Files" FILES ${QRC_SOURCE_FILES})

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
        ${QRC_FILES}
)
# -- customer

find_package(Eigen3 REQUIRED)

find_package(Sophus REQUIRED)

find_package(PCL REQUIRED)

include_directories(
  ${CMAKE_SOURCE_DIR}/../../thirdparty
  ${CMAKE_SOURCE_DIR}/../../src/include
  ${Sophus_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

aux_source_directory(${CMAKE_SOURCE_DIR}/../../src/src SRC_FILES)

# --


if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(QSection
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
        ${SRC_FILES}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET QSection APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(QSection SHARED
            ${PROJECT_SOURCES}
            ${SRC_FILES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(QSection
            ${PROJECT_SOURCES}
            ${SRC_FILES}
        )
    endif()
endif()

target_link_libraries(QSection PRIVATE
  Qt${QT_VERSION_MAJOR}::Widgets
  Qt${QT_VERSION_MAJOR}::DataVisualization
  # --customer
  Sophus::Sophus
  ${PCL_LIBRARIES}
  # --
)


set_target_properties(QSection PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(QSection)
endif()
