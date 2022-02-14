#include <iostream>
#include <random>

#include "artwork/timer/timer.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

namespace ns_chp3 {
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
}  // namespace ns_chp3

int main(int argc, char const *argv[]) {
  ns_chp3::use_eigen();
  return 0;
}
