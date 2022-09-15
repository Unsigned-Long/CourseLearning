//
// Created by csl on 9/12/22.
//
#include "datetime.h"
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
                             unsigned short minute, const ns_spp::BigDouble &sec)
        : year(year), month(month), day(day),
          hour(hour), minute(minute), second(sec) {}

// Julian

ns_spp::Julian::Julian(const std::string &days) : days(days) {}

ns_spp::Julian::Julian(const ns_spp::BigDouble &days) : days(days) {}

// JulianDay

ns_spp::Gregorian ns_spp::JulianDay::toDateTime() const {
    using namespace boost;

    auto iDays = static_cast<int>(this->days);
    auto ymd =
            gregorian::date::calendar_type::from_julian_day_number(iDays);

    auto year = ymd.year;
    auto month = ymd.month;
    auto day = ymd.day;

    auto fracHour = (days - iDays) * ns_spp::BigDouble("24.0");
    auto hour = static_cast<unsigned int>(fracHour);

    auto fracMinute = (fracHour - hour) * ns_spp::BigDouble("60.0");
    auto minute = static_cast<unsigned int>(fracMinute);

    auto second = (fracMinute - minute) * ns_spp::BigDouble("60.0");

    return Gregorian(year, month, day, hour, minute, second);
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

// ModJulianDay

ns_spp::Gregorian ns_spp::ModJulianDay::toDateTime() const {
    using namespace boost;

    auto iDays = static_cast<int>(this->days);
    auto ymd =
            gregorian::date::calendar_type::from_modjulian_day_number(iDays);

    auto year = ymd.year;
    auto month = ymd.month;
    auto day = ymd.day;

    auto fracHour = (days - iDays) * ns_spp::BigDouble("24.0");
    auto hour = static_cast<unsigned int>(fracHour);

    auto fracMinute = (fracHour - hour) * ns_spp::BigDouble("60.0");
    auto minute = static_cast<unsigned int>(fracMinute);

    auto second = (fracMinute - minute) * ns_spp::BigDouble("60.0");

    return Gregorian(year, month, day, hour, minute, second);
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

// GPSTime

ns_spp::ModJulianDay ns_spp::GPSTime::toModJulianDay() const {
    auto days = this->week * BigDouble("7") + this->secOfWeek / BigDouble("86400");
    return ModJulianDay(Config::TimeSystem::GPSTOrigin.days + days);
}

ns_spp::Gregorian ns_spp::GPSTime::toDateTime() const {
    return this->toModJulianDay().toDateTime();
}

ns_spp::JulianDay ns_spp::GPSTime::toJulianDay() const {
    return this->toModJulianDay().toJulianDay();
}

ns_spp::GPSTime::GPSTime(unsigned short week, const std::string &secOfWeek)
        : week(week), secOfWeek(secOfWeek) {}

ns_spp::GPSTime::GPSTime(unsigned short week, const ns_spp::BigDouble &secOfWeek)
        : week(week), secOfWeek(secOfWeek) {}
