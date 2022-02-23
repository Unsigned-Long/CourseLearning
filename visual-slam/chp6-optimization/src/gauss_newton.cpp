#include "artwork/geometry/point.hpp"
#include "artwork/timer/timer.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <random>
#include <vector>
namespace ns_chp6 {
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
} // namespace ns_chp6

int main(int argc, char const *argv[]) {
  ns_chp6::fitting();
  return 0;
}
