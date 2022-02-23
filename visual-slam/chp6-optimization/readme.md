# Chapter 6 : Optimization

___Author : csl___

___E-Mail : 3079625093@qq.com___

[TOC]

## 1. Project Tree

```cpp
.
├── build
├── CMakeLists.txt
├── readme.md
└── src
    ├── gauss_newton.cpp
    └── use_ceres.cpp

11 directories, 37 files
```

## 2. Gauss Newton

### source code

```cpp
void fitting() {
  /**
   * @brief y = exp(ax**2 + bx + c) + e
   * e = y - exp(ax**2 + bx + c)
   *
   * de/da = - x**2 * exp(ax**2 + bx + c)
   * de/db = - x * exp(ax**2 + bx + c)
   * de/dc = - exp(ax**2 + bx + c)
   */
  double ar = 1.0, br = 2.0, cr = 1.0;
  double ae = 2.0, be = -1.0, ce = 5.0;
  double sigma = 1.0;
  std::default_random_engine e;
  std::normal_distribution<> n(0.0, sigma);
  ns_geo::PointSet2d data(100);
  for (int i = 0; i != data.size(); ++i) {
    double x = i / 100.0;
    double y = std::exp(ar * x * x + br * x + cr) + n(e);
    data.at(i).x() = x;
    data.at(i).y() = y;
  }
  ns_timer::Timer<> timer;
  int count = 0;
  while (true) {
    ++count;
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    for (const auto &p : data) {
      double x = p.x();
      double y = p.y();
      double ep = std::exp(ae * x * x + be * x + ce);
      double error = y - ep;
      Eigen::Vector3d J = Eigen::Vector3d::Zero();
      J(0) = -x * x * ep;
      J(1) = -x * ep;
      J(2) = -ep;
      H += J * J.transpose();
      b -= J * error;
    }

    auto delta = H.ldlt().solve(b);
    if (delta.norm() < 1E-6) {
      break;
    } else {
      ae += delta(0);
      be += delta(1);
      ce += delta(2);
    }
  }
  std::cout << timer.last_elapsed("total cost time") << std::endl;
  std::cout << "iter time: " << count << std::endl;
  std::cout << "the result is\nae: " << ae << ", be: " << be << ", ce: " << ce << std::endl;
  return;
}
```

### output

```cpp
{total cost time: 6.227111(ms)}
iter time: 9
the result is
ae: 0.94184, be: 2.09468, ce: 0.965536
```

## 3. Ceres

### source

```cpp
struct FittingCost {
ns_geo::Point2d _point;
FittingCost(double x, double y) : _point(x, y) {}

template <typename T>
bool operator()(const T *const params, T *error) const {
    T a = params[0];
    T b = params[1];
    T c = params[2];
    T x = (T)this->_point.x();
    T y = (T)this->_point.y();
    error[0] = y - ceres::exp(a * x * x + b * x + c);
    return true;
}
};

void fittingCurve() {
double ar = 1.0, br = 2.0, cr = 1.0;
double ae = 2.0, be = -1.0, ce = 5.0;
double sigma = 1.0;
std::default_random_engine e;
std::normal_distribution<> n(0.0, sigma);
ceres::Problem prob;
double param[3] = {ae, be, ce};
for (int i = 0; i != 100; ++i) {
    double x = i / 100.0;
    double y = std::exp(ar * x * x + br * x + cr) + n(e);
    auto fun = new ceres::AutoDiffCostFunction<FittingCost, 1, 3>(new FittingCost(x, y));
    prob.AddResidualBlock(fun, nullptr, param);
}
ceres::Solver::Options op;
op.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
op.minimizer_progress_to_stdout = true;
ceres::Solver::Summary s;
ns_timer::Timer<> timer;
ceres::Solve(op, &prob, &s);
auto cost_time = timer.last_elapsed("cost time");
std::cout << s.BriefReport() << std::endl;
std::cout << "the result is\nae: " << param[0] << ", be: " << param[1] << ", ce: " << param[2] << std::endl;
std::cout << cost_time << std::endl;
return;
}
```



### output

```cpp
iter      cost      cost_change  |gradient|   |step|    tr_ratio  tr_radius  ls_iter  iter_time  total_time
   0  1.599457e+06    0.00e+00    3.52e+06   0.00e+00   0.00e+00  1.00e+04        0    1.95e-03    2.02e-03
   1  1.887477e+05    1.41e+06    4.87e+05   9.86e-01   8.82e-01  1.81e+04        1    2.03e-03    4.14e-03
   2  1.789562e+04    1.71e+05    6.78e+04   9.84e-01   9.05e-01  3.87e+04        1    1.96e-03    6.13e-03
   3  1.097403e+03    1.68e+04    8.60e+03   1.08e+00   9.41e-01  1.16e+05        1    1.59e-03    7.74e-03
   4  8.257147e+01    1.01e+03    6.57e+02   1.47e+00   9.68e-01  3.48e+05        1    1.51e-03    9.28e-03
   5  4.619213e+01    3.64e+01    2.69e+01   1.12e+00   9.95e-01  1.04e+06        1    1.54e-03    1.08e-02
   6  4.569829e+01    4.94e-01    4.64e-01   1.99e-01   1.01e+00  3.13e+06        1    2.21e-03    1.31e-02
   7  4.569793e+01    3.59e-04    3.80e-03   5.71e-03   1.01e+00  9.40e+06        1    1.85e-03    1.50e-02
Ceres Solver Report: Iterations: 8, Initial cost: 1.599457e+06, Final cost: 4.569793e+01, Termination: CONVERGENCE
the result is
ae: 0.941871, be: 2.09463, ce: 0.965554
{cost time: 15.079847(ms)}
```

## 4. GGO

### source code

```cpp
```

### output

```cpp
```



## 5. Cmake

```cmake
cmake_minimum_required(VERSION 3.10)

project(optimization VERSION 1.0)

add_executable(${CMAKE_PROJECT_NAME}_gauss_newton ${CMAKE_SOURCE_DIR}/src/gauss_newton.cpp)

find_package(Ceres)
find_package(Eigen3)

include_directories(${EIGEN_INCLUDE_DIRS})

add_executable(${CMAKE_PROJECT_NAME}_use_ceres ${CMAKE_SOURCE_DIR}/src/use_ceres.cpp)

target_include_directories(${CMAKE_PROJECT_NAME}_use_ceres PRIVATE ${CERES_INCLUDE_DIRS})

target_link_libraries(${CMAKE_PROJECT_NAME}_use_ceres ${CERES_LIBRARIES})
```
