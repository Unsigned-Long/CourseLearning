//
// Created by csl on 9/18/22.
//

#ifndef SPP_TEST_BASE_CAST_HPP
#define SPP_TEST_BASE_CAST_HPP

#include "artwork/logger/logger.h"
#include "utils/base_cast.hpp"
#include "gtest/gtest.h"

TEST(base_convert, BaseConvert) {
    using namespace ns_spp;

    std::string hex = "806F53FA";
    auto dec = BaseCast::toDec<16, unsigned long>(hex);
    LOG_VAR(hex, dec);
    EXPECT_EQ(hex, BaseCast::decTo<16>(dec));

    EXPECT_EQ(BaseCast::decTo<2>(6), "110");
    auto val = BaseCast::toDec<2, int>("110");
    EXPECT_EQ(val, 6);

}

#endif //SPP_TEST_BASE_CAST_HPP
