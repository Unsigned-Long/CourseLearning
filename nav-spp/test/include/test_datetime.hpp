//
// Created by csl on 9/12/22.
//

#ifndef SPP_TEST_DATETIME_HPP
#define SPP_TEST_DATETIME_HPP

#include "gtest/gtest.h"
#include "datetime.h"
#include "boost/date_time/gregorian/gregorian.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"
#include "artwork/logger/logger.h"

TEST(datetime, DateTime) {
    using namespace ns_spp;

    DateTime dt(2000, 6, 30, 0, 0, 0.0);
    EXPECT_EQ(dt.year, 2000);
    EXPECT_EQ(dt.month, 6);
    EXPECT_EQ(dt.day, 30);
    EXPECT_EQ(dt.hour, 0);
    EXPECT_EQ(dt.minute, 0);
    EXPECT_FLOAT_EQ(dt.second, 0.0);
    LOG_VAR(DateTime());

    EXPECT_EQ(dt.toJulianDay().toDateTime(), dt);
    EXPECT_EQ(dt.toModJulianDay().toDateTime(), dt);
}

TEST(datetime, JulianDay) {
    using namespace ns_spp;
    JulianDay julianDay(3333333, 0.33);
    EXPECT_EQ(julianDay.days, 3333333);
    EXPECT_FLOAT_EQ(julianDay.fracDays, 0.33);
    LOG_VAR(julianDay.toDateTime());

    EXPECT_EQ(julianDay.toDateTime().toJulianDay(), julianDay);
}

TEST(datetime, ModJulianDay) {
    using namespace ns_spp;
    ModJulianDay modJulianDay(333333, 0.33);
    EXPECT_EQ(modJulianDay.days, 333333);
    EXPECT_FLOAT_EQ(modJulianDay.fracDays, 0.33);
    LOG_VAR(modJulianDay.toDateTime());

    EXPECT_EQ(modJulianDay.toDateTime().toModJulianDay(), modJulianDay);
}

#endif //SPP_TEST_DATETIME_HPP
