#ifndef ICP_H
#define ICP_H

#include "eigen3/Eigen/Dense"
#include "geometry/point.hpp"
#define FORMAT_VECTOR
#include "logger/logger.h"
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"

namespace ns_section {
  struct ICP {
  public:
    static Sophus::SE3d solve(const ns_geo::PointSet3d &pc1, const ns_geo::PointSet3d &pc2, const ushort iter = 50);

  protected:
    static std::pair<ns_geo::PointSet3d, ns_geo::Point3d> normalize(const ns_geo::PointSet3d &pc);

    static ns_geo::Point3d nearest(const ns_geo::PointSet3d &pc, const ns_geo::Point3d &p);

    static Eigen::Vector3d toVec3d(const ns_geo::Point3d &p);

    static ns_geo::Point3d fromVec3d(const Eigen::Vector3d &vec);
  };
} // namespace ns_section

#endif