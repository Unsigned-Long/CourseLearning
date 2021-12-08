#include "parameter.h"

namespace ns_pygeo
{
#pragma region namespace params
    namespace params
    {

        ns_point::Point3d BLH2XYZ(const ns_point::Point3d &blh, const ns_pygeo::params::Ellipsoid &e)
        {
            ns_point::Point3d xyz;
            auto &b = blh.x();
            auto &l = blh.y();
            auto &h = blh.z();
            auto N = e.N(b);
            xyz.x() = (N + h) * std::cos(b) * std::cos(l);
            xyz.y() = (N + h) * std::cos(b) * std::sin(l);
            xyz.z() = (N * (1 - e.e_fir_2()) + h) * std::sin(b);
            return xyz;
        }

        ns_point::Point3d XYZ2SphCoor(const ns_point::Point3d &xyz)
        {
            ns_point::Point3d p;
            auto &x = xyz.x();
            auto &y = xyz.y();
            auto &z = xyz.z();
            p.x() = std::sqrt(x * x + y * y + z * z);
            p.y() = std::atan2(std::sqrt(x * x + y * y), z);
            p.z() = atan2(y, x);
            return p;
        }

        ns_point::Point3d BLH2SphCoor(const ns_point::Point3d &blh, const ns_pygeo::params::Ellipsoid &e)
        {
            return XYZ2SphCoor(BLH2XYZ(blh, e));
        }

        double degree2radian(double degree)
        {
            return degree * PI / 180.0;
        }

        double radian2degree(double radian)
        {
            return radian * 180.0 / PI;
        }
    } // namespace params
#pragma endregion
} // namespace ns_pygeo
