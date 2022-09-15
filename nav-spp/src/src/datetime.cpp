//
// Created by csl on 9/12/22.
//
#include "datetime.h"

#include <utility>
#include "boost/date_time/gregorian/gregorian.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"

// Gregorian

ns_spp::Gregorian::Gregorian() {
    using namespace boost::gregorian;
    using namespace boost::posix_time;

    ptime now = second_clock::local_time();
    date d = now.date();
    auto daytime = now.time_of_day();

    this->year = d.year();
    this->month = d.month();
    this->day = d.day();

    this->hour = daytime.hours();
    this->minute = daytime.minutes();
    this->second = daytime.seconds();
}

ns_spp::JulianDay ns_spp::Gregorian::toJulianDay() const {
    using namespace boost::gregorian;

    date d(year, month, day);
    auto fracDay = (second / ns_spp::BigDouble("3600.0")
                    + minute / ns_spp::BigDouble("60.0") + hour) / ns_spp::BigDouble("24.0");

    return JulianDay(fracDay + d.julian_day());
}

ns_spp::ModJulianDay ns_spp::Gregorian::toModJulianDay() const {
    using namespace boost::gregorian;

    date d(year, month, day);
    auto fracDay = (second / ns_spp::BigDouble("3600.0")
                    + minute / ns_spp::BigDouble("60.0") + hour) / ns_spp::BigDouble("24.0");

    return ModJulianDay(fracDay + d.modjulian_day());
}

ns_spp::GPSTime ns_spp::Gregorian::toGPSTime() const {
    return this->toModJulianDay().toGPSTime();
}

ns_spp::Gregorian::Gregorian(unsigned short year, unsigned short month, unsigned short day, unsigned short hour,
                             unsigned short minute, const std::string &sec)
        : year(year), month(month), day(day),
          hour(hour), minute(minute), second(sec) {}

ns_spp::Gregorian::Gregorian(unsigned short year, unsigned short month, unsigned short day, unsigned short hour,
                             unsigned short minute, ns_spp::BigDouble sec)
        : year(year), month(month), day(day),
          hour(hour), minute(minute), second(std::move(sec)) {}

ns_spp::BDTime ns_spp::Gregorian::toBDTime() const {
    return this->toModJulianDay().toBDTime();
}

unsigned short ns_spp::Gregorian::dayOfYear() const {
    using namespace boost::gregorian;
    date d(year, month, day);
    return d.day_of_year();
}

// Julian

ns_spp::Julian::Julian(const std::string &days) : days(days) {}

ns_spp::Julian::Julian(ns_spp::BigDouble days) : days(std::move(days)) {}

// JulianDay

ns_spp::Gregorian ns_spp::JulianDay::toGregorian() const {
    using namespace boost;

    auto iDays = static_cast<int>(this->days);
    auto ymd =
            gregorian::date::calendar_type::from_julian_day_number(iDays);

    auto year = ymd.year;
    auto month = ymd.month;
    auto day = ymd.day;

    auto fracHour = (days - iDays) * ns_spp::BigDouble("24.0");
    auto hour = static_cast<unsigned short>(fracHour);

    auto fracMinute = (fracHour - hour) * ns_spp::BigDouble("60.0");
    auto minute = static_cast<unsigned short>(fracMinute);

    auto second = (fracMinute - minute) * ns_spp::BigDouble("60.0");

    return {year, month, day, hour, minute, second};
}

ns_spp::JulianDay::JulianDay() {
    *this = Gregorian().toJulianDay();
}

ns_spp::ModJulianDay ns_spp::JulianDay::toModJulianDay() const {
    return ns_spp::ModJulianDay(this->days - ns_spp::BigDouble("2400000.5"));
}

ns_spp::GPSTime ns_spp::JulianDay::toGPSTime() const {
    return this->toModJulianDay().toGPSTime();
}

ns_spp::JulianDay::JulianDay(const std::string &days) : Julian(days) {}

ns_spp::JulianDay::JulianDay(const ns_spp::BigDouble &days) : Julian(days) {}

ns_spp::BDTime ns_spp::JulianDay::toBDTime() const {
    return this->toModJulianDay().toBDTime();
}

// ModJulianDay

ns_spp::Gregorian ns_spp::ModJulianDay::toGregorian() const {
    using namespace boost;

    auto iDays = static_cast<int>(this->days);
    auto ymd =
            gregorian::date::calendar_type::from_modjulian_day_number(iDays);

    auto year = ymd.year;
    auto month = ymd.month;
    auto day = ymd.day;

    auto fracHour = (days - iDays) * ns_spp::BigDouble("24.0");
    auto hour = static_cast<unsigned short>(fracHour);

    auto fracMinute = (fracHour - hour) * ns_spp::BigDouble("60.0");
    auto minute = static_cast<unsigned short>(fracMinute);

    auto second = (fracMinute - minute) * ns_spp::BigDouble("60.0");

    return {year, month, day, hour, minute, second};
}

ns_spp::ModJulianDay::ModJulianDay() {
    *this = Gregorian().toModJulianDay();
}

ns_spp::JulianDay ns_spp::ModJulianDay::toJulianDay() const {
    return ns_spp::JulianDay(this->days + ns_spp::BigDouble("2400000.5"));
}

ns_spp::GPSTime ns_spp::ModJulianDay::toGPSTime() const {
    auto days = this->days - Config::TimeSystem::GPSTOrigin.days;
    GPSTime gpsTime;
    gpsTime.week = static_cast<unsigned short>(days / BigDouble("7"));
    gpsTime.secOfWeek = (days - gpsTime.week * BigDouble("7")) * BigDouble("86400");
    return gpsTime;
}

ns_spp::ModJulianDay::ModJulianDay(const std::string &days) : Julian(days) {}

ns_spp::ModJulianDay::ModJulianDay(const ns_spp::BigDouble &days) : Julian(days) {}

ns_spp::BDTime ns_spp::ModJulianDay::toBDTime() const {
    auto days = this->days - Config::TimeSystem::BDTOrigin.days;
    BDTime bdTime;
    bdTime.week = static_cast<unsigned short>(days / BigDouble("7"));
    bdTime.secOfWeek = (days - bdTime.week * BigDouble("7")) * BigDouble("86400");
    return bdTime;
}

// GPSTime

ns_spp::ModJulianDay ns_spp::GPSTime::toModJulianDay() const {
    auto days = this->week * BigDouble("7") + this->secOfWeek / BigDouble("86400");
    return ModJulianDay(Config::TimeSystem::GPSTOrigin.days + days);
}

ns_spp::Gregorian ns_spp::GPSTime::toGregorian() const {
    return this->toModJulianDay().toGregorian();
}

ns_spp::JulianDay ns_spp::GPSTime::toJulianDay() const {
    return this->toModJulianDay().toJulianDay();
}

ns_spp::GPSTime::GPSTime(unsigned short week, const std::string &secOfWeek)
        : NavTime(week, secOfWeek) {}

ns_spp::GPSTime::GPSTime(unsigned short week, const ns_spp::BigDouble &secOfWeek)
        : NavTime(week, secOfWeek) {}
// BDTime

ns_spp::BDTime::BDTime(unsigned short week, const std::string &secOfWeek)
        : NavTime(week, secOfWeek) {}

ns_spp::BDTime::BDTime(unsigned short week, const ns_spp::BigDouble &secOfWeek)
        : NavTime(week, secOfWeek) {}

ns_spp::ModJulianDay ns_spp::BDTime::toModJulianDay() const {
    auto days = this->week * BigDouble("7") + this->secOfWeek / BigDouble("86400");
    return ModJulianDay(Config::TimeSystem::BDTOrigin.days + days);
}

ns_spp::Gregorian ns_spp::BDTime::toGregorian() const {
    return this->toModJulianDay().toGregorian();
}

ns_spp::JulianDay ns_spp::BDTime::toJulianDay() const {
    return this->toModJulianDay().toJulianDay();
}

ns_spp::NavTime::NavTime(unsigned short week, const std::string &secOfWeek)
        : week(week), secOfWeek(secOfWeek) {}

ns_spp::NavTime::NavTime(unsigned short week, ns_spp::BigDouble secOfWeek)
        : week(week), secOfWeek(std::move(secOfWeek)) {}