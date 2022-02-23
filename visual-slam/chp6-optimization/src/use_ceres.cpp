#include "artwork/geometry/point.hpp"
#include "artwork/timer/timer.h"
#include <ceres/ceres.h>

namespace ns_chp6 {
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
} // namespace ns_chp6

int main(int argc, char const *argv[]) {
  ns_chp6::fittingCurve();
  return 0;
}
