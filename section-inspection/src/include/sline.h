#ifndef SLINE_H
#define SLINE_H

#include "geometry/point.hpp"

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

    // Fit lines from points
    static SLine fit(const ns_geo::PointSet2d &pts);

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