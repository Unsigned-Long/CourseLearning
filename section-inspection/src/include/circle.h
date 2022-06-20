#ifndef CIRCLE_H
#define CIRCLE_H

#include "geometry/point.hpp"

namespace ns_section {
  struct Circle {
  public:
    /**
     * @brief the members
     */
    ns_geo::Point2d cen;
    double rad;

  public:
    /**
     * @brief construct a new Circle object
     */
    Circle(const ns_geo::Point2d &cen, const double &rad) : cen(cen), rad(rad) {}
    /**
     * @brief Construct a new Circle object from three points
     */
    Circle(const ns_geo::Point2d &p1, const ns_geo::Point2d &p2, const ns_geo::Point2d &p3);

    // Calculate the distance from a point to the nearest point on the circle
    double distance(const ns_geo::Point2d &p) const;

    // find the nearest point on the circle
    ns_geo::Point2d nearest(const ns_geo::Point2d &p) const;

    // using gauss-newton method to fit a circle
    static Circle fit(const ns_geo::PointSet2d &points, const ushort iter = 10);

    // using ransac method to find a circle
    static Circle ransac(const ns_geo::PointSet2d &points, const ushort num = 10);
  };
  /**
   * @brief override operator '<<' for type 'Circle'
   */
  std::ostream &operator<<(std::ostream &os, const Circle &obj);

} // namespace ns_section

#endif