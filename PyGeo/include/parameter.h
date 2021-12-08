#pragma once

#include <iostream>
#include <cmath>
#include "point.h"

namespace ns_pygeo
{
#pragma region namespace params
    namespace params
    {
        const double PI = std::atan(1.0) * 4.0;

        constexpr double G = 6.67259E-11;

        class Ellipsoid
        {
        private:
            // semimajor axis of the ellipsoid [m]
            const double _a;
            // flattening of the ellipsoid [1]
            const double _f;
            // geocentric gravitational constant of the earth(including the atmosphere) [m^3 * s^(-2)]
            const double _GM;
            // angular velocity of the earth [rad * s^(-1)]
            const double _omega;

            // spherical-harmonic coeﬃcients
            const double _J2;
            const double _J4;
            const double _J6;
            const double _J8;

        public:
            Ellipsoid() = delete;
            Ellipsoid(double a, double f, double GM, double omega,
                      double J2, double J4, double J6, double J8)
                : _a(a), _f(f), _GM(GM), _omega(omega), _J2(J2), _J4(J4), _J6(J6), _J8(J8) {}

        public:
            inline double a() const { return this->_a; }
            inline double f() const { return this->_f; }
            inline double GM() const { return this->_GM; }
            inline double omega() const { return this->_omega; }
            inline double J2() const { return this->_J2; }
            inline double J4() const { return this->_J4; }
            inline double J6() const { return this->_J6; }
            inline double J8() const { return this->_J8; }
            // semiminor axis of the ellipsoid [m]
            inline double b() const { return this->_a * (1.0 - this->_f); }
            // first eccentricity [1]
            inline double e_fir() const { return std::sqrt(_a * _a - b() * b()) / _a; }
            // first eccentricity squared [1]
            inline double e_fir_2() const { return (_a * _a - b() * b()) / (_a * _a); }
            // second eccentricity [1]
            inline double e_sed() const { return std::sqrt(_a * _a - b() * b()) / b(); }
            // second eccentricity squared [1]
            inline double e_sed_2() const { return (_a * _a - b() * b()) / (b() * b()); }
            // linear eccentricity [1]
            inline double E() const { return std::sqrt(_a * _a - b() * b()); }
            // polar radius of curvature [m]
            inline double c() const { return _a * _a / b(); }
            // normal potential at the ellipsoid [m^2 * s^(-2)]
            inline double U_0() const { return _GM / E() * std::atan(E() / b()) + _omega * _omega * _a * _a / 3.0; }
            // normal gravity at the equator [m * s^(-2)]
            inline double gamma_a() const
            {
                double m = _omega * _omega * _a * _a * b() / _GM;
                return _GM / (_a * b()) * (1.0 - 1.5 * m - 3.0 * e_sed_2() * m / 14.0);
            }
            // normal gravity at the pole [m * s^(-2)]
            inline double gamma_b() const
            {
                double m = _omega * _omega * _a * _a * b() / _GM;
                return _GM / (_a * _a) * (1.0 + m + 3.0 * e_sed_2() * m / 7.0);
            }
            // m = std::pow(omega * a, 2) * b / GM
            inline double m() const { return _omega * _omega * _a * _a * b() / _GM; }
            // mass of the earth (includes atmosphere) [kg]
            inline double M() const { return _GM / G; }
            // the normal gravity [phi radian]
            inline double normalGravity(double radian_phi) const
            {
                auto v1 = _a * std::cos(radian_phi) * std::cos(radian_phi);
                auto v2 = b() * std::sin(radian_phi) * std::sin(radian_phi);
                return (v1 * gamma_a() + v2 * gamma_b()) / std::sqrt(v1 * _a + v2 * b());
            }
            // the normal gravity [phi radian]
            inline double gamma(double radian_phi) const { return normalGravity(radian_phi); }

            inline double W(double radian_B) const { return std::sqrt(1 - this->e_fir_2() * std::sin(radian_B) * std::sin(radian_B)); }

            inline double V(double radian_B) const
            {
                auto W = this->W(radian_B);
                return (1 + this->e_sed_2()) * W * W;
            }

            inline double N(double radian_B) const { return this->_a / this->W(radian_B); }
        };

        // the WGS 84 reference ellipsoid
        const Ellipsoid WGS_84(6378137.000, 1.0 / 298.257223563, 3986004.418E08, 7292115.000E-11,
                               108263.000E-08, -0.00000237091222, 0.00000000608347, -0.00000000001427);

        // the GRS 1980 reference ellipsoid
        const Ellipsoid GRS_80(6378137.000, 1.0 / 298.257222101, 3986005.000E08, 7292115.000E-11,
                               108263.000E-08, -0.00000237091222, 0.00000000608347, -0.00000000001427);

        ns_point::Point3d BLH2XYZ(const ns_point::Point3d &blh, const ns_pygeo::params::Ellipsoid &e);

        ns_point::Point3d XYZ2SphCoor(const ns_point::Point3d &xyz);

        ns_point::Point3d BLH2SphCoor(const ns_point::Point3d &blh, const ns_pygeo::params::Ellipsoid &e);

        double degree2radian(double degree);

        double radian2degree(double radian);
    } // namespace params
#pragma endregion
} // namespace ns_pygeo
