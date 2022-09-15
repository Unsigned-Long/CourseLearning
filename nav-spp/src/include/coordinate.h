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

namespace ns_spp {

    struct PointXYZ {
    public:
        double X;
        double Y;
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
        double B;
        double L;
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
        double a;
        double b;
        double c;
        double f;
        double efir;
        double efir2;
        double esed;
        double esed2;

    public:
        RefEllipsoid(double majorAxis, double flattening);

        double W(double latitude) const;

        double V(double latitude) const;

        double N(double latitude) const;

        double M(double latitude) const;

        friend std::ostream &operator<<(std::ostream &os, const RefEllipsoid &ellipsoid) {
            os << "RefEllipsoid['a': " << ellipsoid.a << ", 'b': " << ellipsoid.b << ", 'c': "
               << ellipsoid.c << ", 'f': " << ellipsoid.f << ", efir: " << ellipsoid.efir
               << ", 'efir2': " << ellipsoid.efir2 << ", 'esed': " << ellipsoid.esed
               << ", 'esed2': " << ellipsoid.esed2;
            return os;
        }

        PointXYZ BLH2XYZ(const PointBLH &p) const;

        PointBLH XYZ2BLH(const PointXYZ &p) const;

        bool operator==(const RefEllipsoid &rhs) const {
            return std::abs(a - rhs.a) < Config::Threshold::DOUBLE_EQ &&
                   std::abs(b - rhs.b) < Config::Threshold::DOUBLE_EQ &&
                   std::abs(c - rhs.c) < Config::Threshold::DOUBLE_EQ &&
                   std::abs(f - rhs.f) < Config::Threshold::DOUBLE_EQ &&
                   std::abs(efir - rhs.efir) < Config::Threshold::DOUBLE_EQ &&
                   std::abs(efir2 - rhs.efir2) < Config::Threshold::DOUBLE_EQ &&
                   std::abs(esed - rhs.esed) < Config::Threshold::DOUBLE_EQ &&
                   std::abs(esed2 - rhs.esed2) < Config::Threshold::DOUBLE_EQ;
        }

        bool operator!=(const RefEllipsoid &rhs) const {
            return !(rhs == *this);
        }
    };
}


#endif //SPP_COORDINATE_H
