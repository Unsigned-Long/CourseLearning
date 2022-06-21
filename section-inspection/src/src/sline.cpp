#include "sline.h"
#include "eigen3/Eigen/Dense"
#include "logger/logger.h"

namespace ns_section {

  SLine SLine::fit(const ns_geo::PointSet2d &pts) {
    // using svd to find the init value
    Eigen::MatrixXd A(pts.size(), 3);
    for (int i = 0; i != pts.size(); ++i) {
      A(i, 0) = pts[i].x;
      A(i, 1) = pts[i].y;
      A(i, 2) = 1.0;
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    Eigen::Matrix3d vMatrix = svd.matrixV();
    Eigen::Vector3d param = vMatrix.col(vMatrix.cols() - 1);
    // if the sline is Vertical, than gauss newton is useless
    if (std::abs(param(1)) < 1E-10) {
      return SLine(param(0), param(1), param(2));
    }
    // gauss newton [assign init value]
    double k = -param(0) / param(1), b = -param(2) / param(0);
    for (int i = 0; i != 10; ++i) {
      Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
      Eigen::Vector2d g = Eigen::Vector2d::Zero();
      for (const auto &p : pts) {
        double x = p.x, y = p.y;
        double d = (k * x - y + b) / std::sqrt(k * k + 1.0);
        double jk = (x * (k * k + 1.0) - k * (k * x - y + b)) / std::pow(k * k + 1, 1.5);
        double jb = 1.0 / std::sqrt(k * k + 1.0);
        Eigen::Vector2d j(jk, jb);
        H += j * j.transpose();
        g -= j * d;
      }
      auto delta = H.ldlt().solve(g);
      k += delta(0);
      b += delta(1);
    }
    return SLine(k, -1.0, b);
  }

  ns_geo::Point2d SLine::nearest(const ns_geo::Point2d &p) const {
    Eigen::Matrix2d A;
    A(0, 0) = b, A(0, 1) = -a;
    A(1, 0) = a, A(1, 1) = b;

    Eigen::Vector2d l;
    l(0) = b * p.x - a * p.y;
    l(1) = -c;

    Eigen::Vector2d np = A.inverse() * l;
    return ns_geo::Point2d(np(0), np(1));
  }

  double SLine::distance(const ns_geo::Point2d &p) const {
    double v1 = std::abs(a * p.x + b * p.y + c);
    double v2 = std::sqrt(a * a + b * b);
    return v1 / v2;
  }

  std::ostream &operator<<(std::ostream &os, const SLine &obj) {
    os << '{';
    os << "'a': " << obj.a << ", 'b': " << obj.b << ", 'c': " << obj.c;
    os << '}';
    return os;
  }

} // namespace ns_section
