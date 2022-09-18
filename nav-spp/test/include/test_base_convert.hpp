//
// Created by csl on 9/18/22.
//

#ifndef SPP_TEST_BASE_CONVERT_HPP
#define SPP_TEST_BASE_CONVERT_HPP

#include "artwork/logger/logger.h"
#include "base_convert.hpp"
#include "gtest/gtest.h"

TEST(base_convert, BaseConvert) {
    using namespace ns_spp;

    std::string hex = "806F53FA";
    auto dec = BaseConvert::toDec<BaseConvert::Base::HEX, unsigned long>(hex);
    LOG_VAR(hex, dec);
    EXPECT_EQ(hex, BaseConvert::decTo<BaseConvert::HEX>(dec));

    EXPECT_EQ(BaseConvert::decTo<BaseConvert::BIT>(6), "110");
    auto val = BaseConvert::toDec<BaseConvert::BIT, unsigned long>("110");
    EXPECT_EQ(val, 6);

}

#endif //SPP_TEST_BASE_CONVERT_HPP