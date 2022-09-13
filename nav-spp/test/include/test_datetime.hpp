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
#include "config.h"

TEST(datetime, DateTime) {
    using namespace ns_spp;
    Config::loadConfigure("../config/spp.json");

    BigDouble second("0");

    DateTime dt(2000, 6, 30, 0, 0, second);

    EXPECT_EQ(dt.year, 2000);
    EXPECT_EQ(dt.month, 6);
    EXPECT_EQ(dt.day, 30);
    EXPECT_EQ(dt.hour, 0);
    EXPECT_EQ(dt.minute, 0);
    EXPECT_EQ(dt.second, second);

    LOG_VAR(DateTime());

    EXPECT_EQ(dt.toJulianDay().toDateTime(), dt);
    EXPECT_EQ(dt.toModJulianDay().toDateTime(), dt);
}

TEST(datetime, JulianDay) {
    using namespace ns_spp;

    JulianDay julianDay(BigDouble("3333333"));

    EXPECT_EQ(julianDay.days, BigDouble("3333333"));

    LOG_VAR(julianDay);

    EXPECT_EQ(julianDay.toDateTime().toJulianDay(), julianDay);
    EXPECT_EQ(julianDay.toModJulianDay().toJulianDay(), julianDay);
}

TEST(datetime, ModJulianDay) {
    using namespace ns_spp;

    ModJulianDay modJulianDay(BigDouble("333333"));

    EXPECT_EQ(modJulianDay.days, BigDouble("333333"));

    LOG_VAR(modJulianDay);

    EXPECT_EQ(modJulianDay.toDateTime().toModJulianDay(), modJulianDay);
    EXPECT_EQ(modJulianDay.toJulianDay().toModJulianDay(), modJulianDay);
}

TEST(datetime, GPSTime) {
    using namespace ns_spp;

    EXPECT_EQ(Config::TimeSystem::GPSTOrigin, ModJulianDay(BigDouble("44244")));

    auto gpsTime = GPSTime(2000, BigDouble("2000"));

    EXPECT_EQ(gpsTime.toDateTime().toGPSTime(), gpsTime);
    EXPECT_EQ(gpsTime.toModJulianDay().toGPSTime(), gpsTime);
    EXPECT_EQ(gpsTime.toJulianDay().toGPSTime(), gpsTime);
}

#endif //SPP_TEST_DATETIME_HPP
