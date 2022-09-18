//
// Created by csl on 9/18/22.
//

#ifndef SPP_TEST_DATA_PARSER_HPP
#define SPP_TEST_DATA_PARSER_HPP

#include "data_parser.h"

#include "gtest/gtest.h"

TEST(data_parser, PortIdentifier) {
    using namespace ns_spp;
    EXPECT_EQ(EnumCast::enumToInteger(PortIdentifier::COM3_31), 127);
    EXPECT_EQ(EnumCast::enumToInteger(PortIdentifier::IMU_ALL), 21);
    EXPECT_EQ(EnumCast::enumToInteger(PortIdentifier::NCOM2_ALL), 27);
    EXPECT_EQ(EnumCast::enumToInteger(PortIdentifier::ICOM4_ALL), 29);
    EXPECT_EQ(EnumCast::enumToInteger(PortIdentifier::THISPORT_ALL), 6);
}

TEST(data_parser, TimeStatus) {
    using namespace ns_spp;
    EXPECT_EQ(EnumCast::enumToInteger(TimeStatus::APPROXIMATE), 60);
    EXPECT_EQ(EnumCast::enumToInteger(TimeStatus::FREEWHEELING), 130);
    EXPECT_EQ(EnumCast::enumToInteger(TimeStatus::FINE), 160);
    EXPECT_EQ(EnumCast::enumToInteger(TimeStatus::SATTIME), 200);
    EXPECT_EQ(EnumCast::enumToInteger(TimeStatus::UNKNOWN), 20);
}

#endif //SPP_TEST_DATA_PARSER_HPP
