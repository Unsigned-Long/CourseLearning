//
// Created by csl on 9/13/22.
//

#ifndef SPP_COORDINATE_H
#define SPP_COORDINATE_H

#include <cmath>
#include <iomanip>
#include <iostream>
#include <tuple>
#include "config.h"
#include "ceres/ceres.h"

namespace ns_spp {

    struct PointXYZ {
    public:
        // x axis [m]
        double X;
        // y axis [m]
        double Y;
        // z axis [m]
        double Z;

        PointXYZ(double x, double y, double z);

        bool operator==(const PointXYZ &rhs) const {
            return std::abs(X - rhs.X) < Config::Threshold::POSITION &&
                   std::abs(Y - rhs.Y) < Config::Threshold::POSITION &&
                   std::abs(Z - rhs.Z) < Config::Threshold::POSITION;
        }

        bool operator!=(const PointXYZ &rhs) const {
            return !(rhs == *this);
        }

        friend std::ostream &operator<<(std::ostream &os, const PointXYZ &xyz) {
            os << "PointXYZ['X': " << xyz.X << ", 'Y': " << xyz.Y << ", 'Z': " << xyz.Z << ']';
            return os;
        }
    };

    struct PointBLH {
    public:
        // latitude [rad]
        double B;
        // longitude [rad]
        double L;
        // height [m]
        double H;

        PointBLH(double b, double l, double h);

        bool operator==(const PointBLH &rhs) const {
            return std::abs(B - rhs.B) < Config::Threshold::POSITION &&
                   std::abs(L - rhs.L) < Config::Threshold::POSITION &&
                   std::abs(H - rhs.H) < Config::Threshold::POSITION;
        }

        bool operator!=(const PointBLH &rhs) const {
            return !(rhs == *this);
        }

        friend std::ostream &operator<<(std::ostream &os, const PointBLH &blh) {
            os << "PointBLH['B': " << blh.B << ", 'L': " << blh.L << ", 'H': " << blh.H << ']';
            return os;
        }
    };

    struct RefEllipsoid {
    public:
        // semi major axis of the ellipsoid
        double a;
        // semi minor axis of the ellipsoid
        double b;
        // polar radius of curvature
        double c;
        // flattening
        double f;
        // first eccentricity
        double eFir;
        // first eccentricity squared
        double eFir2;
        // second eccentricity
        double eSed;
        // second eccentricity squared
        double eSed2;

    public:
        RefEllipsoid(double majorAxis, double flattening);

        double W(double latitude) const;

        double V(double latitude) const;

        /**
         * Radius of curvature of prime vertical
         */
        double N(double latitude) const;

        /*
        * Radius of curvature of meridian circle
        */
        double M(double latitude) const;

        friend std::ostream &operator<<(std::ostream &os, const RefEllipsoid &ellipsoid) {
            os << "RefEllipsoid['a': " << ellipsoid.a << ", 'b': " << ellipsoid.b << ", 'c': "
               << ellipsoid.c << ", 'f': " << ellipsoid.f << ", eFir: " << ellipsoid.eFir
               << ", 'eFir2': " << ellipsoid.eFir2 << ", 'eSed': " << ellipsoid.eSed
               << ", 'eSed2': " << ellipsoid.eSed2;
            return os;
        }

        /*
         * convert geodetic coordinates of a point to rectangular coordinates
         */
        PointXYZ BLH2XYZ(const PointBLH &p) const;

        /**
         * convert rectangular coordinates of a point to geodetic coordinates
         */
        PointBLH XYZ2BLH(const PointXYZ &p) const;

        bool operator==(const RefEllipsoid &rhs) const {
            return std::abs(a - rhs.a) < Config::Threshold::DOUBLE_EQ &&
                   std::abs(b - rhs.b) < Config::Threshold::DOUBLE_EQ &&
                   std::abs(c - rhs.c) < Config::Threshold::DOUBLE_EQ &&
                   std::abs(f - rhs.f) < Config::Threshold::DOUBLE_EQ &&
                   std::abs(eFir - rhs.eFir) < Config::Threshold::DOUBLE_EQ &&
                   std::abs(eFir2 - rhs.eFir2) < Config::Threshold::DOUBLE_EQ &&
                   std::abs(eSed - rhs.eSed) < Config::Threshold::DOUBLE_EQ &&
                   std::abs(eSed2 - rhs.eSed2) < Config::Threshold::DOUBLE_EQ;
        }

        bool operator!=(const RefEllipsoid &rhs) const {
            return !(rhs == *this);
        }

    };

}


#endif //SPP_COORDINATE_H
