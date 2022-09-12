//
// Created by csl on 9/12/22.
//
#include "datetime.h"
#include "boost/date_time/gregorian/gregorian.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"

std::ostream &ns_spp::operator<<(std::ostream &os, const ns_spp::DateTime &dateTime) {
    os << "year: " << dateTime.year << " month: " << dateTime.month << " day: " << dateTime.day << " hour: "
       << dateTime.hour << " minute: " << dateTime.minute << " second: " << dateTime.second;
    return os;
}

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
    long double fracDay = (second / 3600.0 + minute / 60.0 + hour) / 24.0;

    return {d.julian_day(), fracDay};
}

ns_spp::DateTime ns_spp::DateTime::fromJulianDay(const ns_spp::JulianDay &julianDay) {
    using namespace boost::gregorian;

    auto now = day_clock::local_day();
    auto d = now - days(now.julian_day() - julianDay.days);

    DateTime dt(0, 0, 0);
    dt.year = d.year();
    dt.month = d.month();
    dt.day = d.day();

    long double fracHour = julianDay.fracDays * 24.0;
    dt.hour = static_cast<unsigned int>(fracHour);

    long double fracMinute = (fracHour - dt.hour) * 60.0;
    dt.minute = static_cast<unsigned int>(fracMinute);

    dt.second = (fracMinute - dt.minute) * 60.0;

    return dt;
}

ns_spp::ModJulianDay ns_spp::DateTime::toModJulianDay() const {
    using namespace boost::gregorian;

    date d(year, month, day);
    long double fracDay = (second / 3600.0 + minute / 60.0 + hour) / 24.0;

    return {d.modjulian_day(), fracDay};
}

ns_spp::DateTime ns_spp::DateTime::fromModJulianDay(const ns_spp::ModJulianDay &modJulianDay) {

    using namespace boost::gregorian;
    auto now = day_clock::local_day();
    auto d = now - days(now.modjulian_day() - modJulianDay.days);

    DateTime dt(0, 0, 0);
    dt.year = d.year();
    dt.month = d.month();
    dt.day = d.day();

    long double fracHour = modJulianDay.fracDays * 24.0;
    dt.hour = static_cast<unsigned int>(fracHour);

    long double fracMinute = (fracHour - dt.hour) * 60.0;
    dt.minute = static_cast<unsigned int>(fracMinute);

    dt.second = (fracMinute - dt.minute) * 60.0;

    return dt;
}

bool ns_spp::DateTime::operator==(const ns_spp::DateTime &rhs) const {
    return year == rhs.year &&
           month == rhs.month &&
           day == rhs.day &&
           hour == rhs.hour &&
           minute == rhs.minute &&
           std::abs(second - rhs.second) < 1E-12;
}

bool ns_spp::DateTime::operator!=(const ns_spp::DateTime &rhs) const {
    return !(rhs == *this);
}

bool ns_spp::JulianDay::operator==(const JulianDay &rhs) const {
    return days == rhs.days && std::abs(fracDays - rhs.fracDays) < 1E-12;
}

bool ns_spp::JulianDay::operator!=(const JulianDay &rhs) const {
    return !(rhs == *this);
}

bool ns_spp::ModJulianDay::operator==(const ModJulianDay &rhs) const {
    return !(rhs == *this);
}

bool ns_spp::ModJulianDay::operator!=(const ModJulianDay &rhs) const {

}

ns_spp::DateTime ns_spp::JulianDay::toDateTime() {
    return DateTime::fromJulianDay(*this);
}

ns_spp::JulianDay ns_spp::JulianDay::fromDateTime(const DateTime &dateTime) {
    return dateTime.toJulianDay();
}

ns_spp::JulianDay::JulianDay() {
    *this = DateTime().toJulianDay();
}

ns_spp::DateTime ns_spp::ModJulianDay::toDateTime() {
    return DateTime::fromModJulianDay(*this);
}

ns_spp::ModJulianDay ns_spp::ModJulianDay::fromDateTime(const ns_spp::DateTime &dateTime) {
    return dateTime.toModJulianDay();
}

ns_spp::ModJulianDay::ModJulianDay() {
    *this = DateTime().toModJulianDay();
}

