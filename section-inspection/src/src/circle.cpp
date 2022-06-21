#include "circle.h"
#include "eigen3/Eigen/Dense"
#include "sampling.hpp"

namespace ns_section {
  Circle::Circle(const ns_geo::Point2d &p1, const ns_geo::Point2d &p2, const ns_geo::Point2d &p3) {
    // move origin
    double x2 = p2.x - p1.x, y2 = p2.y - p1.y;
    double x3 = p3.x - p1.x, y3 = p3.y - p1.y;
    double r2 = x2 * x2 + y2 * y2, r3 = x3 * x3 + y3 * y3;
    // compute center
    cen.x = (y3 * r2 - y2 * r3) / (2.0 * (x2 * y3 - y2 * x3)) + p1.x;
    cen.y = (x2 * r3 - x3 * r2) / (2.0 * (x2 * y3 - y2 * x3)) + p1.y;
    // compute radius
    rad = ns_geo::distance(cen, p1);
  }

  double Circle::distance(const ns_geo::Point2d &p) const {
    return ns_geo::distance(nearest(p), p);
  }

  ns_geo::Point2d Circle::nearest(const ns_geo::Point2d &p) const {
    // compute angle
    double deltaX = p.x - cen.x, deltaY = p.y - cen.y;
    double theta = std::atan2(deltaY, deltaX);
    if (deltaY < 0.0) {
      theta += 2.0 * M_PI;
    }
    // compute point on circle
    ns_geo::Point2d target;
    target.x = rad * std::cos(theta) + cen.x;
    target.y = rad * std::sin(theta) + cen.y;
    return target;
  }

  Circle Circle::ransac(const ns_geo::PointSet2d &points, const ushort num) {
    std::vector<std::pair<Circle, double>> samples;
    std::default_random_engine engine;
    for (int i = 0; i != num; ++i) {
      auto vec = ns_section::samplingWoutReplace2(engine, points, 3);
      // construct the circle and compute the error
      Circle curCir(vec[0], vec[1], vec[2]);
      double error = 0.0;
      for (const auto &p : points) {
        error += curCir.distance(p);
      }
      samples.push_back({curCir, error});
    }
    // find the circle whose error is min
    auto iter = std::min_element(samples.cbegin(), samples.cend(),
                                 [](const std::pair<Circle, double> &p1, const std::pair<Circle, double> &p2) {
                                   return p1.second < p2.second;
                                 });
    return iter->first;
  }

  Circle Circle::fit(const ns_geo::PointSet2d &points, const ushort iter) {
    // get init parameters
    auto cir = Circle::ransac(points);
    for (int i = 0; i != iter; ++i) {
      // get matrix
      Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
      Eigen::Vector3d g = Eigen::Vector3d::Zero();
      for (const auto &p : points) {
        double dis = ns_geo::distance(p, cir.cen);
        double error = dis - cir.rad;
        Eigen::Vector3d j(-(p.x - cir.cen.x) / dis, -(p.y - cir.cen.y) / dis, -1.0);
        H += j * j.transpose();
        g -= j * error;
      }
      // solve
      Eigen::Vector3d delta = H.ldlt().solve(g);
      // update
      cir.cen.x += delta(0);
      cir.cen.y += delta(1);
      cir.rad += delta(2);
    }
    return cir;
  }
  
  std::ostream &operator<<(std::ostream &os, const Circle &obj) {
    os << '{';
    os << "'cen': " << obj.cen << ", 'rad': " << obj.rad;
    os << '}';
    return os;
  }

} // namespace ns_section
