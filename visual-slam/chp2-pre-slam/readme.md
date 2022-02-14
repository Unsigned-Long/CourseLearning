# chp2-pre-slam

___Author : csl___

___E-Mail : 3079625093@qq.com___

[TOC]

## 1. Project Structure

<center>total 17 directories, 57 files</center>

the file system of this project named '[preslam]()':

+ [.vscode](./.vscode) contains the configure files for VSCode
+ [bin](./bin) the binary output directory
  + [preslam](./bin/preslam)
+ [build](./build) the cmake binary directory for build
+ [include](./include) the include directory contains cpp head files
  + [helloslam.h](./include/helloslam.h)
+ [lib](./lib) the lib output directory
  + [libhelloslam.so](./lib/libhelloslam.so)
+ [src](./src) directory constains cpp source files
  + [helloslam.cpp](./src/helloslam.cpp)
+ [.gitignore](./.gitignore) the gitignore file
+ [CMakeLists.txt](./CMakeLists.txt) the cmake file
+ [main.cpp](./main.cpp) the main source file
+ [readme.md](./readme.md) the readme file

## 2. Main Apis

### 1. CPP

+ ___void hello_slam()___

```cpp
/**
 * @brief a no-return function to print "hello, slam!"
 */
```

### 2. CMake

+ ```cmake
  # Sets the minimum required version of cmake
  cmake_minimum_required(VERSION 3.10)
  ```

+ ```cmake
  # Set the name of the project
  project(preslam VERSION 0.0.1)
  ```

+ ```cmake
  # Add include directories to the build
  include_directories(${CMAKE_SOURCE_DIR}/include)
  ```

+ ```cmake
  # Add a library to the project using the 'helloslam.cpp'
  add_library(${LIB_NAME} SHARED ${CMAKE_SOURCE_DIR}/src/helloslam.cpp)
  ```

+ ```cmake
  # Add an executable to the project using the 'main.cpp'
  add_executable(${CMAKE_PROJECT_NAME} ${CMAKE_SOURCE_DIR}/main.cpp)
  ```

+ ```cmake
  # link the lib to the executable
  target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE  ${LIB_NAME})
  ```

## 3. Source Code

### 1. CPP

+ [helloslam.h](./include/helloslam.h)

```cpp
#pragma once

#include <iostream>

namespace ns_chp2 {
/**
 * @brief a no-return function to print "hello, slam!"
 */
void hello_slam();
}  // namespace ns_chp2
```

+ [helloslam.cpp](./src/helloslam.cpp)

```cpp
#include "helloslam.h"

namespace ns_chp2 {
void hello_slam() {
  // print "Hello, slam!"
  std::cout << "Hello, slam!" << std::endl;
  return;
}
}  // namespace ns_chp2
```

+ [main.cpp](./main.cpp)

```cpp
#include "helloslam.h"

int main(int argc, char const *argv[]) {
  // call 'hello_slam'
  ns_chp2::hello_slam();
  return 0;
}
```

### 2. CMake

+ [CMakeLists.txt](./CMakeLists.txt)

```cmake
# Sets the minimum required version of cmake
cmake_minimum_required(VERSION 3.10)

# Set the name of the project
project(preslam VERSION 0.0.1)

# Add include directories to the build
include_directories(${CMAKE_SOURCE_DIR}/include)

# Set a environment variable to the lib name
set(LIB_NAME helloslam)

# set the lib output directory
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

# set the bin output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# Add a library to the project using the 'helloslam.cpp'
add_library(${LIB_NAME} SHARED ${CMAKE_SOURCE_DIR}/src/helloslam.cpp)

# Add an executable to the project using the 'main.cpp'
add_executable(${CMAKE_PROJECT_NAME} ${CMAKE_SOURCE_DIR}/main.cpp)

# link the lib to the executable
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE  ${LIB_NAME})
```

