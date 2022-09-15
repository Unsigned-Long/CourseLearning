//
// Created by csl on 9/12/22.
//

#ifndef SPP_DATETIME_H
#define SPP_DATETIME_H

#include <ostream>
#include <boost/multiprecision/cpp_dec_float.hpp>
#include "config.h"

namespace ns_spp {
    struct Gregorian;
    struct Julian;
    struct JulianDay;
    struct ModJulianDay;
    struct GPSTime;

    using BigDouble = boost::multiprecision::cpp_dec_float_50;

    struct Gregorian {
        unsigned short year;
        unsigned short month;
        unsigned short day;
        unsigned short hour;
        unsigned short minute;
        BigDouble second;

        Gregorian(unsigned short year, unsigned short month, unsigned short day,
                  unsigned short hour = 0, unsigned short minute = 0,
                  const std::string &sec = "0.0");

        Gregorian(unsigned short year, unsigned short month, unsigned short day,
                  unsigned short hour, unsigned short minute,
                  const BigDouble &sec);

        Gregorian();

        JulianDay toJulianDay() const;

        ModJulianDay toModJulianDay() const;

        GPSTime toGPSTime() const;

    public:
        friend std::ostream &operator<<(std::ostream &os, const Gregorian &dateTime) {
            os << "Gregorian['y': " << dateTime.year << ", 'mon': " << dateTime.month << ", 'd': " << dateTime.day
               << ", 'h': " << dateTime.hour << ", 'min': " << dateTime.minute << ", 's': " << dateTime.second << ']';
            return os;
        }

        bool operator==(const ns_spp::Gregorian &rhs) const {
            return year == rhs.year &&
                   month == rhs.month &&
                   day == rhs.day &&
                   hour == rhs.hour &&
                   minute == rhs.minute &&
                   std::abs(static_cast<long double>(second - rhs.second)) < Config::Threshold::DOUBLE_EQ;
        }

        bool operator!=(const ns_spp::Gregorian &rhs) const {
            return !(rhs == *this);
        }

    };

    struct Julian {
    public:
        BigDouble days;

    protected:

        explicit Julian(const std::string &days);

        explicit Julian(const BigDouble &days);

        Julian() = default;

    public:

        bool operator==(const Julian &rhs) const {
            return std::abs(static_cast<long double>(days - rhs.days)) < Config::Threshold::DOUBLE_EQ;
        }

        bool operator!=(const Julian &rhs) const {
            return !(rhs == *this);
        }

        virtual Gregorian toDateTime() const = 0;
    };

    struct JulianDay : public Julian {

    public:

        explicit JulianDay(const std::string &days = "0.0");

        explicit JulianDay(const BigDouble &days);

        JulianDay();

        Gregorian toDateTime() const override;

        ModJulianDay toModJulianDay() const;

        GPSTime toGPSTime() const;

        friend std::ostream &operator<<(std::ostream &os, const JulianDay &day) {
            os << "JulianDay['d': " << day.days << "]";
            return os;
        }
    };

    struct ModJulianDay : public Julian {

    public:

        explicit ModJulianDay(const std::string &days = "0.0");

        explicit ModJulianDay(const BigDouble &days);

        ModJulianDay();

        Gregorian toDateTime() const override;

        JulianDay toJulianDay() const;

        GPSTime toGPSTime() const;

        friend std::ostream &operator<<(std::ostream &os, const ModJulianDay &day) {
            os << "ModJulianDay['d': " << day.days << "]";
            return os;
        }
    };

    struct GPSTime {
    public:
        unsigned short week;
        BigDouble secOfWeek;

        static GPSTime origin;

        explicit GPSTime(unsigned short week = 0, const std::string &secOfWeek = "0.0");

        explicit GPSTime(unsigned short week, const BigDouble &secOfWeek);

        ModJulianDay toModJulianDay() const;

        Gregorian toDateTime() const;

        JulianDay toJulianDay() const;

        friend std::ostream &operator<<(std::ostream &os, const GPSTime &gpsTime) {
            os << "GPSTime['w': " << gpsTime.week << ", 'sow': " << gpsTime.secOfWeek << ']';
            return os;
        }

        bool operator==(const ns_spp::GPSTime &rhs) const {
            return week == rhs.week &&
                   std::abs(static_cast<long double>(secOfWeek - rhs.secOfWeek)) < Config::Threshold::DOUBLE_EQ;
        }

        bool operator!=(const ns_spp::GPSTime &rhs) const {
            return !(rhs == *this);
        }

    };

}

#endif //SPP_DATETIME_H
