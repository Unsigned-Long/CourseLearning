# Chp3-rigid-motion

___Aurther: csl___

___E-Mail : 3079625093@qq.com___

[TOC]

## 1. Project Structure

```cpp
.
├── bin
│   ├── rigidmotion_case
│   ├── rigidmotion_eigen
│   └── rigidmotion_geometry
├── build
├── CMakeLists.txt
├── readme.md
└── src
    ├── part1-use-eigen
    │   ├── main.cpp
    │   └── output.log
    ├── part2-use-geometry
    │   ├── main.cpp
    │   └── output.log
    └── part3-case
        ├── main.cpp
        └── output.log

23 directories, 71 files
```

## 2. part1-use-eigen

### 1. [source code](./src/part1-use-eigen/)

```cpp
void use_eigen() {
  Eigen::Matrix<float, 2, 3> mat_23;

  Eigen::Vector<double, 3> vec_3;

  Eigen::Matrix<float, 3, 1> mat_31;

  Eigen::MatrixX3d mat_33 = Eigen::Matrix3d::Zero();

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> mat_dyn;

  Eigen::MatrixXd mat_xd;

  mat_23 << 1, 2, 3, 4, 5, 6;

  std::cout << "mat_23: \n" << mat_23 << std::endl;

  std::cout << "print mat_23 through 'for-loop':\n";
  for (int i = 0; i != mat_23.rows(); ++i) {
    for (int j = 0; j != mat_23.cols(); ++j) std::cout << mat_23(i, j) << ' ';
    std::cout << std::endl;
  }

  vec_3 << 3, 2, 1;
  mat_31 << 4, 5, 6;

  std::cout << "vec_3: \n" << vec_3 << std::endl;
  std::cout << "mat_31: \n" << mat_31 << std::endl;

  Eigen::Matrix<double, 2, 1> result = mat_23.cast<double>() * vec_3;
  std::cout << "result for 'mat_23 * vec_3':\n"
            << result.transpose() << std::endl;

  Eigen::Matrix<float, 2, 1> result2 = mat_23 * mat_31;
  std::cout << "result for 'mat_23 * mat_31':\n"
            << result2.transpose() << std::endl;

  mat_33 = Eigen::Matrix3d::Random();

  std::cout << "mat_33 [random]:\n" << mat_33 << std::endl;
  std::cout << "mat_33 [trans]:\n" << mat_33.transpose() << std::endl;
  std::cout << "mat_33 [sum]:\n" << mat_33.sum() << std::endl;
  std::cout << "mat_33 [trace]:\n" << mat_33.trace() << std::endl;
  std::cout << "mat_33 [times 10]:\n" << 10 * mat_33 << std::endl;
  std::cout << "mat_33 [inv]:\n" << mat_33.inverse() << std::endl;
  std::cout << "mat_33 [det]:\n" << mat_33.determinant() << std::endl;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(
      mat_33.transpose() * mat_33);
  std::cout << "eigenvalues:\n" << eigen_solver.eigenvalues() << std::endl;
  std::cout << "eigenvectors:\n" << eigen_solver.eigenvectors() << std::endl;

  std::default_random_engine e;
  e.seed(time_t());
  std::uniform_int_distribution<> u(2, 10);
  auto dime = u(e);
  Eigen::MatrixXd mat_nn = Eigen::MatrixXd::Random(dime, dime);
  mat_nn = mat_nn.transpose() * mat_nn;

  Eigen::MatrixXd v_n = Eigen::MatrixXd::Random(dime, 1);

  std::cout << "mat_nn:\n" << mat_nn << std::endl;
  std::cout << "v_n:" << v_n.transpose() << std::endl;

  ns_timer::Timer<> timer;

  Eigen::MatrixXd x = mat_nn.inverse() * v_n;

  std::cout << "x: " << x.transpose() << std::endl;
  std::cout << timer.last_elapsed("cost time for 'mat_nn * v_n'") << std::endl;

  x = mat_nn.colPivHouseholderQr().solve(v_n);

  std::cout << "x: " << x.transpose() << std::endl;
  std::cout << timer.last_elapsed("cost time for 'mat_nn * v_n'") << std::endl;

  x = mat_nn.ldlt().solve(v_n);

  std::cout << "x: " << x.transpose() << std::endl;
  std::cout << timer.last_elapsed("cost time for 'mat_nn * v_n'") << std::endl;

  return;
}
```

### 2. [output](./src/part1-use-eigen/output.log)

```cpp
mat_23: 
1 2 3
4 5 6
print mat_23 through 'for-loop':
1 2 3 
4 5 6 
vec_3: 
3
2
1
mat_31: 
4
5
6
result for 'mat_23 * vec_3':
10 28
result for 'mat_23 * mat_31':
32 77
mat_33 [random]:
 0.680375   0.59688 -0.329554
-0.211234  0.823295  0.536459
 0.566198 -0.604897 -0.444451
mat_33 [trans]:
 0.680375 -0.211234  0.566198
  0.59688  0.823295 -0.604897
-0.329554  0.536459 -0.444451
mat_33 [sum]:
1.61307
mat_33 [trace]:
1.05922
mat_33 [times 10]:
 6.80375   5.9688 -3.29554
-2.11234  8.23295  5.36459
 5.66198 -6.04897 -4.44451
mat_33 [inv]:
-0.198521   2.22739    2.8357
  1.00605 -0.555135  -1.41603
 -1.62213   3.59308   3.28973
mat_33 [det]:
0.208598
eigenvalues:
0.0242899
 0.992154
  1.80558
eigenvectors:
-0.549013 -0.735943  0.396198
 0.253452 -0.598296 -0.760134
-0.796459  0.316906 -0.514998
mat_nn:
0.0136946 0.0400457
0.0400457  0.139564
v_n:0.0268018  0.904459
x: -105.585  36.7766
{cost time for 'mat_nn * v_n': 0.134178(ms)}
x: -105.585  36.7766
{cost time for 'mat_nn * v_n': 0.207775(ms)}
x: -105.585  36.7766
{cost time for 'mat_nn * v_n': 0.132776(ms)}
```

## 3. part2-use-geometry

### 1. [source code](./src/part2-use-geometry/main.cpp)

```cpp
void use_geometry() {
  Eigen::Matrix3d rot_mat = Eigen::Matrix3d::Identity();
  Eigen::AngleAxisd rot_vec(M_PI / 4, Eigen::Vector3d(0, 0, 1));
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "rot_vec:\n" << rot_vec.matrix() << std::endl;
  rot_mat = rot_vec.toRotationMatrix();

  Eigen::Vector3d v(1, 0, 0);
  std::cout << "(" << v.transpose() << ") rotated by 'angle axisd': ("
            << (rot_vec * v).transpose() << ")\n";

  std::cout << "(" << v.transpose() << ") rotated by 'rotate matrix': ("
            << (rot_mat * v).transpose() << ")\n";

  Eigen::Vector3d euler_angle = rot_mat.eulerAngles(2, 1, 0);

  std::cout << "(yaw[z] pitch[y] roll[x]): (" << euler_angle.transpose()
            << ")\n";

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.rotate(rot_vec);
  T.pretranslate(Eigen::Vector3d(1, 3, 4));
  std::cout << "transform matrix:\n" << T.matrix() << std::endl;

  std::cout << "(" << v.transpose() << ") rotated by 'transform': ("
            << (T * v).transpose() << ")\n";

  Eigen::Quaterniond q(rot_mat);
  std::cout << "Quaternion: " << q.coeffs().transpose() << std::endl;

  std::cout << "(" << v.transpose() << ") rotated by 'Quaternion': ("
            << (q * v).transpose() << ")\n";
  return;
}
```



### 2. [output](./src/part2-use-geometry/output.log)

```cpp
rot_vec:
 0.707 -0.707  0.000
 0.707  0.707  0.000
 0.000  0.000  1.000
(1.000 0.000 0.000) rotated by 'angle axisd': (0.707 0.707 0.000)
(1.000 0.000 0.000) rotated by 'rotate matrix': (0.707 0.707 0.000)
(yaw[z] pitch[y] roll[x]): ( 0.785 -0.000  0.000)
transform matrix:
 0.707 -0.707  0.000  1.000
 0.707  0.707  0.000  3.000
 0.000  0.000  1.000  4.000
 0.000  0.000  0.000  1.000
(1.000 0.000 0.000) rotated by 'transform': (1.707 3.707 4.000)
Quaternion: 0.000 0.000 0.383 0.924
(1.000 0.000 0.000) rotated by 'Quaternion': (0.707 0.707 0.000)
```

## 4. part3-case

### 1. [source code](./src/part3-case/main.cpp)

```cpp
void exp_case() {
  Eigen::Quaterniond q1(0.35, 0.2, 0.3, 0.1);
  q1.normalize();
  Eigen::Vector3d t1(0.3, 0.1, 0.1);
  Eigen::Isometry3d T1(q1);
  T1.pretranslate(t1);

  Eigen::Quaterniond q2(-0.5, 0.4, -0.1, 0.2);
  q2.normalize();
  Eigen::Vector3d t2(-0.1, 0.5, 0.3);
  Eigen::Isometry3d T2(q2);
  T2.pretranslate(t2);

  Eigen::Vector3d p(0.5, 0, 0.2);
  Eigen::Vector3d result = T2 * T1.inverse() * p;

  std::cout << "the result is: " << result.transpose() << std::endl;
  return;
}
```

### 2. [output](./src/part3-case/output.log)

```cpp
the result is: -0.0309731    0.73499   0.296108
```

## 5. Cmake

```cmake
cmake_minimum_required(VERSION 3.10)

project(rigidmotion)

find_package(Eigen3)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

add_executable(${CMAKE_PROJECT_NAME}_eigen ${CMAKE_SOURCE_DIR}/src/part1-use-eigen/main.cpp)

add_executable(${CMAKE_PROJECT_NAME}_geometry ${CMAKE_SOURCE_DIR}/src/part2-use-geometry/main.cpp)

add_executable(${CMAKE_PROJECT_NAME}_case ${CMAKE_SOURCE_DIR}/src/part3-case/main.cpp)

target_include_directories(${CMAKE_PROJECT_NAME}_eigen PRIVATE ${EIGEN_INCLUDE_DIRS})

target_include_directories(${CMAKE_PROJECT_NAME}_geometry PRIVATE ${EIGEN_INCLUDE_DIRS})

target_include_directories(${CMAKE_PROJECT_NAME}_case PRIVATE ${EIGEN_INCLUDE_DIRS})
```

