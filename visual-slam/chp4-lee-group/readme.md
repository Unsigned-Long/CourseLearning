# chp4-lee-group

___Author : csl___

___E-Mail : 3079625093@qq.com___

[TOC]

## 1. Project Tree

```cpp
.
├── build
├── CMakeLists.txt
├── output
│   └── log.log
├── readme.md
└── src
    └── leegroup.cpp

12 directories, 35 files
```

## 2. Code

### 1. source code

```cpp
void leegroup() {
  double angle = M_PI / 2.0;
  Eigen::Vector3d axis(0, 0, 1);
  Eigen::Matrix3d rot_mat = Eigen::AngleAxisd(angle, axis).toRotationMatrix();

  Sophus::SO3d so3(rot_mat);

  std::cout << "so3.log(): " << so3.log().transpose() << std::endl;
  std::cout << "hat: \n" << Sophus::SO3d::hat(so3.log()) << std::endl;
  std::cout << "vee: "
            << Sophus::SO3d::vee(Sophus::SO3d::hat(so3.log())).transpose()
            << std::endl;
  std::cout << "exp(so3.log()): \n"
            << Sophus::SO3d::exp(so3.log()).matrix() << std::endl;

  Eigen::Matrix3d update_so3(Eigen::AngleAxisd(1E-10, axis).toRotationMatrix());

  std::cout << "update exp(so3.log()): \n"
            << (Sophus::SO3d::exp(Sophus::SO3d(update_so3).log()) *
                Sophus::SO3d::exp(so3.log()))
                   .matrix()
            << std::endl;

  Sophus::SE3d se3(rot_mat, Eigen::Vector3d(3, 4, 5));
  std::cout << "se3.log(): " << se3.log().transpose() << std::endl;
  std::cout << "hat: \n" << Sophus::SE3d::hat(se3.log()) << std::endl;
  std::cout << "vee: "
            << Sophus::SE3d::vee(Sophus::SE3d::hat(se3.log())).transpose()
            << std::endl;
  std::cout << "exp(se3.log()): \n"
            << Sophus::SE3d::exp(se3.log()).matrix() << std::endl;

  Sophus::SE3d update_se3(Eigen::AngleAxisd(1E-10, axis).toRotationMatrix(),
                          Eigen::Vector3d(3, 4, 5 + 1E-10));

  std::cout << "update exp(se3.log()): \n"
            << (Sophus::SE3d::exp(Sophus::SE3d(update_se3).log()) *
                Sophus::SE3d::exp(se3.log()))
                   .matrix()
            << std::endl;
  return;
}
```



### 2. output

```cpp
so3.log():      0      0 1.5708
hat: 
      0 -1.5708       0
 1.5708       0      -0
     -0       0       0
vee:      0      0 1.5708
exp(so3.log()): 
2.22045e-16          -1           0
          1 2.22045e-16           0
          0           0           1
update exp(so3.log()): 
-1e-10     -1      0
     1 -1e-10      0
     0      0      1
se3.log():  5.49779 0.785398        5        0        0   1.5708
hat: 
       0  -1.5708        0  5.49779
  1.5708        0       -0 0.785398
      -0        0        0        5
       0        0        0        0
vee:  5.49779 0.785398        5        0        0   1.5708
exp(se3.log()): 
2.22045e-16          -1           0           3
          1 2.22045e-16           0           4
          0           0           1           5
          0           0           0           1
update exp(se3.log()): 
-1e-10     -1      0      6
     1 -1e-10      0      8
     0      0      1     10
     0      0      0      1
```

### 3. cmake

```cmake
cmake_minimum_required(VERSION 3.10)

project(leegroup VERSION 1.0)

find_package(Sophus)
find_package(Eigen3)

set (CMAKE_DISABLE_FIND_PACKAGE_fmt ON)

add_executable(${CMAKE_PROJECT_NAME} ${CMAKE_SOURCE_DIR}/src/leegroup.cpp)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${SOPHUS_INCLUDE_DIRS})

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${EIGEN3_INCLUDE_DIRS})

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE Sophus::Sophus)
```

