//
// Created by csl on 9/12/22.
//

#ifndef SPP_TEST_CONFIG_HPP
#define SPP_TEST_CONFIG_HPP

#include "gtest/gtest.h"
#include "config.h"

TEST(config, Config) {
    using namespace ns_spp;

    Config config = Config::loadConfigure("../config/spp.json");

    EXPECT_EQ(config.author.name, "unsigned-long2");
    EXPECT_EQ(config.author.e_mail, "3079625093@qq.com");
}

#endif //SPP_TEST_CONFIG_HPP
