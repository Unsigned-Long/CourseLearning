#include "point3d.h"
namespace ns_gps {
  
  std::ostream &operator<<(std::ostream &os, const Point3d &obj) {
    os << '{';
    os << "'x': " << obj.x << ", 'y': " << obj.y << ", 'z': " << obj.z;
    os << '}';
    return os;
  }
} // namespace ns_gps
