#ifndef POINT3D_H
#define POINT3D_H

#include <iostream>

namespace ns_gps {

  struct Point3d {
  public:
    /**
     * @brief the members
     */
    double x;
    double y;
    double z;

  public:
    /**
     * @brief construct a new Point3d object
     */
    Point3d(const double &x, const double &y, const double &z)
        : x(x), y(y), z(z) {}
  };
  /**
   * @brief override operator '<<' for type 'Point3d'
   */
  std::ostream &operator<<(std::ostream &os, const Point3d &obj);

} // namespace ns_gps

#endif