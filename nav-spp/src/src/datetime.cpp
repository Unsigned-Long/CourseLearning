//
// Created by csl on 9/12/22.
//
#include "datetime.h"
#include "boost/date_time/gregorian/gregorian.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"
#include "config.h"


// DateTime

ns_spp::DateTime::DateTime() {
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

ns_spp::JulianDay ns_spp::DateTime::toJulianDay() const {
    using namespace boost::gregorian;

    date d(year, month, day);
    auto fracDay = (second / ns_spp::BigDouble("3600.0")
                    + minute / ns_spp::BigDouble("60.0") + hour) / ns_spp::BigDouble("24.0");

    return JulianDay(fracDay + d.julian_day());
}

ns_spp::ModJulianDay ns_spp::DateTime::toModJulianDay() const {
    using namespace boost::gregorian;

    date d(year, month, day);
    auto fracDay = (second / ns_spp::BigDouble("3600.0")
                    + minute / ns_spp::BigDouble("60.0") + hour) / ns_spp::BigDouble("24.0");

    return ModJulianDay(fracDay + d.modjulian_day());
}

ns_spp::GPSTime ns_spp::DateTime::toGPSTime() const {
    return this->toModJulianDay().toGPSTime();
}

// JulianDay

ns_spp::DateTime ns_spp::JulianDay::toDateTime() const {
    using namespace boost;

    auto now = gregorian::day_clock::local_day();
    auto iDays = static_cast<unsigned int>(this->days);
    auto d = now - gregorian::days(now.julian_day() - iDays);

    DateTime dt(0, 0, 0);
    dt.year = d.year();
    dt.month = d.month();
    dt.day = d.day();

    auto fracHour = (days - iDays) * ns_spp::BigDouble("24.0");
    dt.hour = static_cast<unsigned int>(fracHour);

    auto fracMinute = (fracHour - dt.hour) * ns_spp::BigDouble("60.0");
    dt.minute = static_cast<unsigned int>(fracMinute);

    dt.second = (fracMinute - dt.minute) * ns_spp::BigDouble("60.0");

    return dt;
}

ns_spp::JulianDay::JulianDay() {
    *this = DateTime().toJulianDay();
}

ns_spp::ModJulianDay ns_spp::JulianDay::toModJulianDay() const {
    return ns_spp::ModJulianDay(this->days - ns_spp::BigDouble("2400000.5"));
}

ns_spp::GPSTime ns_spp::JulianDay::toGPSTime() const {
    return this->toModJulianDay().toGPSTime();
}

// ModJulianDay

ns_spp::DateTime ns_spp::ModJulianDay::toDateTime() const {
    using namespace boost;

    auto now = gregorian::day_clock::local_day();
    auto iDays = static_cast<unsigned int>(this->days);
    auto d = now - gregorian::days(now.modjulian_day() - iDays);

    DateTime dt(0, 0, 0);
    dt.year = d.year();
    dt.month = d.month();
    dt.day = d.day();

    auto fracHour = (days - iDays) * ns_spp::BigDouble("24.0");
    dt.hour = static_cast<unsigned int>(fracHour);

    auto fracMinute = (fracHour - dt.hour) * ns_spp::BigDouble("60.0");
    dt.minute = static_cast<unsigned int>(fracMinute);

    dt.second = (fracMinute - dt.minute) * ns_spp::BigDouble("60.0");

    return dt;
}

ns_spp::ModJulianDay::ModJulianDay() {
    *this = DateTime().toModJulianDay();
}

ns_spp::JulianDay ns_spp::ModJulianDay::toJulianDay() const {
    return ns_spp::JulianDay(this->days + ns_spp::BigDouble("2400000.5"));
}

ns_spp::GPSTime ns_spp::ModJulianDay::toGPSTime() const {
    auto days = this->days - Config::TimeSystem::GPSTOrigin.days;
    GPSTime gpsTime;
    gpsTime.week = static_cast<unsigned short >(days / BigDouble("7"));
    gpsTime.secOfWeek = (days - gpsTime.week * BigDouble("7")) * BigDouble("86400");
    return gpsTime;
}

// GPSTime

ns_spp::ModJulianDay ns_spp::GPSTime::toModJulianDay() const {
    auto days = this->week * BigDouble("7") + this->secOfWeek / BigDouble("86400");
    return ModJulianDay(Config::TimeSystem::GPSTOrigin.days + days);
}

ns_spp::DateTime ns_spp::GPSTime::toDateTime() const {
    return this->toModJulianDay().toDateTime();
}

ns_spp::JulianDay ns_spp::GPSTime::toJulianDay() const {
    return this->toModJulianDay().toJulianDay();
}

