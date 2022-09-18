//
// Created by csl on 9/18/22.
//

#ifndef SPP_TEST_ENUM_CAST_HPP
#define SPP_TEST_ENUM_CAST_HPP

#include "gtest/gtest.h"
#include "enum_cast.hpp"

enum class Color {
    RED = 12, GREEN = 14, BLUE = 19
};

TEST(enum_cast, EnumCast) {
    using namespace ns_spp;
    Color color = Color::GREEN;

    EXPECT_EQ(EnumCast::enumToInteger(color), 14);
    EXPECT_EQ(EnumCast::enumToString<Color>(color), "GREEN");

    EXPECT_EQ(EnumCast::integerToEnum<Color>(14), color);
    EXPECT_EQ(EnumCast::stringToEnum<Color>("GREEN"), color);

    EXPECT_EQ(EnumCast::stringToInteger<Color>("GREEN"), 14);
    EXPECT_EQ(EnumCast::integerToString<Color>(14), "GREEN");
}

#endif //SPP_TEST_ENUM_CAST_HPP
