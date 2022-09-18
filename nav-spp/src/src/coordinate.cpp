//
// Created by csl on 9/13/22.
//

#include "coordinate.h"

// PointXYZ

ns_spp::PointXYZ::PointXYZ(double x, double y, double z) : X(x), Y(y), Z(z) {}

// PointBLH
ns_spp::PointBLH::PointBLH(double b, double l, double h) : B(b), L(l), H(h) {}

// RefEllipsoid
double ns_spp::RefEllipsoid::W(double latitude) const {
    return std::sqrt(1.0 - eFir2 * std::pow(std::sin(latitude), 2));
}

double ns_spp::RefEllipsoid::V(double latitude) const {
    return std::sqrt(1.0 + eSed2 * std::pow(std::cos(latitude), 2));
}

double ns_spp::RefEllipsoid::N(double latitude) const {
    return a / W(latitude);
}

double ns_spp::RefEllipsoid::M(double latitude) const {
    return a * (1 - eFir2) / std::pow(W(latitude), 3);
}

ns_spp::RefEllipsoid::RefEllipsoid(double majorAxis, double flattening) : a(majorAxis), f(flattening) {
    b = (1.0 - f) * a;
    c = a * a / b;
    eFir = std::sqrt(a * a - b * b) / a;
    eFir2 = eFir * eFir;
    eSed = std::sqrt(a * a - b * b) / b;
    eSed2 = eSed * eSed;
}

ns_spp::PointXYZ ns_spp::RefEllipsoid::BLH2XYZ(const ns_spp::PointBLH &p) const {

    double B = p.B, L = p.L, H = p.H;

    double N = this->N(B);
    double X = (N + H) * cos(B) * cos(L);
    double Y = (N + H) * cos(B) * sin(L);
    double Z = (N * (1.0 - eFir2) + H) * sin(B);

    return {static_cast<double>(X), static_cast<double>(Y), static_cast<double>(Z)};
}

ns_spp::PointBLH ns_spp::RefEllipsoid::XYZ2BLH(const ns_spp::PointXYZ &p) const {

    double X = p.X, Y = p.Y, Z = p.Z;
    double rxy = std::sqrt(X * X + Y * Y);
    double L = std::atan2(Y, X);
    double deltaZ = eFir2 * Z;
    double B = std::atan2(Z, rxy);
    while (true) {
        double newB = std::atan(
                (Z + N(B) * eFir2 * std::sin(B)) / rxy
        );
        double deltaB = std::abs(B - newB);
        B = newB;
        if (deltaB < ns_spp::Config::Threshold::ITERATE) {
            break;
        }
    }
    double H = rxy / std::cos(B) - N(B);
    return {B, L, H};
}
