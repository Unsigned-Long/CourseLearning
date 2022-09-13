//
// Created by csl on 9/13/22.
//

#ifndef SPP_COORDINATE_H
#define SPP_COORDINATE_H

#include <cmath>
#include <iomanip>
#include <iostream>
#include <tuple>

namespace ns_spp {

    struct PointXYZ {
    public:
        double X;
        double Y;
        double Z;

        PointXYZ(double x, double y, double z);

        bool operator==(const PointXYZ &rhs) const;

        bool operator!=(const PointXYZ &rhs) const;

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

        bool operator==(const PointBLH &rhs) const;

        bool operator!=(const PointBLH &rhs) const;

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
        RefEllipsoid(double longRadius, double oblateness);

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

        bool operator==(const RefEllipsoid &rhs) const;

        bool operator!=(const RefEllipsoid &rhs) const;
    };
}


#endif //SPP_COORDINATE_H
