#include "sline.h"
#include "logger/logger.h"

namespace ns_section {

  SLine SLine::ransac(const ns_geo::PointSet2d &pts, const ushort iter) {
    std::vector<std::pair<SLine, double>> lines;
    std::default_random_engine e;
    for (int i = 0; i != iter; ++i) {
      auto tp = ns_geo::samplingWoutReplace(e, pts, 2);
      SLine l(pts[tp[0]], pts[tp[1]]);
      double error = 0.0;
      for (const auto &p : pts) {
        error += l.distance(p);
      }
      lines.push_back({l, error});
    }
    auto tar = std::min_element(lines.cbegin(), lines.cend(),
                                [](const std::pair<SLine, double> &p1, const std::pair<SLine, double> &p2) {
                                  return p1.second < p2.second;
                                });
    return tar->first;
  }

  SLine SLine::fit(const ns_geo::PointSet2d &pts) {
    // ransac
    SLine l = SLine::ransac(pts);
    // gauss newton [assign init value]
    double k = -l.a / l.b, b = -l.c / l.b;
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
