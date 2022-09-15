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
    return std::sqrt(1.0 - efir2 * std::pow(std::sin(latitude), 2));
}

double ns_spp::RefEllipsoid::V(double latitude) const {
    return std::sqrt(1.0 + esed2 * std::pow(std::cos(latitude), 2));
}

double ns_spp::RefEllipsoid::N(double latitude) const {
    return a / W(latitude);
}

double ns_spp::RefEllipsoid::M(double latitude) const {
    return a * (1 - efir2) / std::pow(W(latitude), 3);
}

ns_spp::RefEllipsoid::RefEllipsoid(double majorAxis, double flattening) : a(majorAxis), f(flattening) {
    b = (1.0 - f) * a;
    c = a * a / b;
    efir = std::sqrt(a * a - b * b) / a;
    efir2 = efir * efir;
    esed = std::sqrt(a * a - b * b) / b;
    esed2 = esed * esed;
}

ns_spp::PointXYZ ns_spp::RefEllipsoid::BLH2XYZ(const ns_spp::PointBLH &p) const {

    double B = p.B, L = p.L, H = p.H;

    double N = this->N(B);
    double X = (N + H) * cos(B) * cos(L);
    double Y = (N + H) * cos(B) * sin(L);
    double Z = (N * (1.0 - efir2) + H) * sin(B);

    return {static_cast<double>(X), static_cast<double>(Y), static_cast<double>(Z)};
}

ns_spp::PointBLH ns_spp::RefEllipsoid::XYZ2BLH(const ns_spp::PointXYZ &p) const {

    double X = p.X, Y = p.Y, Z = p.Z;
    double rxy = std::sqrt(X * X + Y * Y);
    double L = std::atan2(Y, X);
    double deltaZ = efir2 * Z;
    double B, H;
    while (true) {
        double sinB = (Z + deltaZ) / std::sqrt(X * X + Y * Y + (Z + deltaZ) * (Z + deltaZ));
        B = std::asin(sinB);
        auto newDeltaZ = this->N(B) * efir2 * sinB;
        double update = std::abs(deltaZ - newDeltaZ);
        deltaZ = newDeltaZ;
        if (update < Config::Threshold::ITERATE) {
            break;
        }
    }
    B = std::atan2(Z + deltaZ, rxy);
    H = std::sqrt(X * X + Y * Y + (Z + deltaZ) * (Z + deltaZ)) - this->N(B);
    return {B, L, H};
}
