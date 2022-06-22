#ifndef SLINE_H
#define SLINE_H

#include "eigen3/Eigen/Dense"
#include "geometry/point.hpp"
#include "geometry/sample.hpp"


namespace ns_section {
  struct SLine {
  public:
    /**
     * @brief the members
     */
    double a;
    double b;
    double c;

  public:
    /**
     * @brief construct a new SLine object
     */
    SLine(const double &a, const double &b, const double &c) {
      // normalize
      double norm = std::sqrt(a * a + b * b + c * c);
      this->a = a / norm;
      this->b = b / norm;
      this->c = c / norm;
    }

    SLine(const ns_geo::Point2d &p1, const ns_geo::Point2d &p2) {
      Eigen::MatrixXd A(2, 3);
      A(0, 0) = p1.x, A(0, 1) = p1.y, A(0, 2) = 1;
      A(1, 0) = p2.x, A(1, 1) = p2.y, A(1, 2) = 1;
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
      Eigen::Matrix3d vMatrix = svd.matrixV();
      Eigen::Vector3d param = vMatrix.col(vMatrix.cols() - 1);
      a = param(0), b = param(1), c = param(2);
    }

    // Fit lines from points
    static SLine fit(const ns_geo::PointSet2d &pts);

    static SLine ransac(const ns_geo::PointSet2d &pts, const ushort iter = 10);

    // Solve the nearest point from the target point to the line
    ns_geo::Point2d nearest(const ns_geo::Point2d &p) const;

    // Solve the distance from the point to the line
    double distance(const ns_geo::Point2d &p) const;
  };
  /**
   * @brief override operator '<<' for type 'SLine'
   */
  std::ostream &operator<<(std::ostream &os, const SLine &obj);

} // namespace ns_section

#endif