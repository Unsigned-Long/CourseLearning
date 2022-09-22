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

TEST(datetime, Gregorian) {
    using namespace ns_spp;

    BigDouble second("0");

    Gregorian dt(2000, 6, 30, 0, 0, second);

    EXPECT_EQ(dt.year, 2000);
    EXPECT_EQ(dt.month, 6);
    EXPECT_EQ(dt.day, 30);
    EXPECT_EQ(dt.hour, 0);
    EXPECT_EQ(dt.minute, 0);
    EXPECT_EQ(dt.second, second);

    LOG_VAR(Gregorian());

    EXPECT_EQ(static_cast<int>(dt.toJulianDay().days), 2451726);
    EXPECT_EQ(static_cast<int>(dt.toModJulianDay().days), 51725);

    EXPECT_EQ(dt.toJulianDay().toGregorian(), dt);
    EXPECT_EQ(dt.toModJulianDay().toGregorian(), dt);

    EXPECT_EQ(dt.dayOfYear(), 182);
}

TEST(datetime, JulianDay) {
    using namespace ns_spp;

    JulianDay julianDay(BigDouble("2451726"));

    EXPECT_EQ(julianDay.days, BigDouble("2451726"));

    LOG_VAR(julianDay);

    EXPECT_EQ(julianDay.toGregorian().toJulianDay(), julianDay);
    EXPECT_EQ(julianDay.toModJulianDay().toJulianDay(), julianDay);
}

TEST(datetime, ModJulianDay) {
    using namespace ns_spp;

    ModJulianDay modJulianDay(BigDouble("51725"));

    EXPECT_EQ(modJulianDay.days, BigDouble("51725"));

    LOG_VAR(modJulianDay);

    EXPECT_EQ(modJulianDay.toGregorian().toModJulianDay(), modJulianDay);
    EXPECT_EQ(modJulianDay.toJulianDay().toModJulianDay(), modJulianDay);
}

TEST(datetime, GPSTime) {
    using namespace ns_spp;

    EXPECT_EQ(Config::TimeSystem::GPSTOrigin, ModJulianDay(BigDouble("44244")));

    auto gpsTime = GPSTime(2000, BigDouble("2000"));
    EXPECT_EQ(gpsTime.week, 2000);
    EXPECT_EQ(gpsTime.secOfWeek, BigDouble("2000"));

    LOG_VAR(gpsTime);

    EXPECT_EQ(gpsTime.toGregorian().toGPSTime(), gpsTime);
    EXPECT_EQ(gpsTime.toModJulianDay().toGPSTime(), gpsTime);
    EXPECT_EQ(gpsTime.toJulianDay().toGPSTime(), gpsTime);
}

TEST(datetime, BDTime) {
    using namespace ns_spp;

    EXPECT_EQ(Config::TimeSystem::BDTOrigin, ModJulianDay(BigDouble("53736")));

    auto bdTime = BDTime(2000, BigDouble("2000"));
    EXPECT_EQ(bdTime.week, 2000);
    EXPECT_EQ(bdTime.secOfWeek, BigDouble("2000"));

    LOG_VAR(bdTime);

    EXPECT_EQ(bdTime.toGregorian().toBDTime(), bdTime);
    EXPECT_EQ(bdTime.toModJulianDay().toBDTime(), bdTime);
    EXPECT_EQ(bdTime.toJulianDay().toBDTime(), bdTime);
}

#endif //SPP_TEST_DATETIME_HPP
