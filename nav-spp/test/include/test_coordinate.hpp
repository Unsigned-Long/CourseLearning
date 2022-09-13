//
// Created by csl on 9/13/22.
//

#ifndef SPP_TEST_COORDINATE_HPP
#define SPP_TEST_COORDINATE_HPP

#include "coordinate.h"
#include "gtest/gtest.h"
#include "artwork/angle/angle.h"
#include "config.h"

TEST(coordinate, PointXYZ) {
    using namespace ns_spp;

    PointXYZ p(1.0, 2.0, 3.0);
    EXPECT_EQ(p.X, 1.0);
    EXPECT_EQ(p.Y, 2.0);
    EXPECT_EQ(p.Z, 3.0);
}

TEST(coordinate, PointBLH) {
    using namespace ns_spp;
    PointBLH p(1.0, 2.0, 3.0);
    EXPECT_EQ(p.B, 1.0);
    EXPECT_EQ(p.L, 2.0);
    EXPECT_EQ(p.H, 3.0);
}

TEST(coordinate, RefEllipsoid) {
    using namespace ns_spp;
    auto p_blh = PointBLH(
            ns_angle::Angle::make_pangle(30, 14, 56.6),
            ns_angle::Angle::make_pangle(120, 44, 23.2),
            22.4
    );
    auto p_xyz = Config::RefEllipsoid::WGS1984.BLH2XYZ(p_blh);
    EXPECT_DOUBLE_EQ(-2818639.053721131756902, p_xyz.X);
    EXPECT_DOUBLE_EQ(4739630.509334281086922, p_xyz.Y);
    EXPECT_DOUBLE_EQ(3194264.984859094955027, p_xyz.Z);

    EXPECT_EQ(Config::RefEllipsoid::WGS1984.XYZ2BLH(p_xyz), p_blh);

}

#endif //SPP_TEST_COORDINATE_HPP
