//
// Created by csl on 9/12/22.
//

#ifndef SPP_DATETIME_H
#define SPP_DATETIME_H

#include <ostream>

namespace ns_spp {
    struct Julian;
    struct JulianDay;
    struct ModJulianDay;

    struct DateTime {
        unsigned short year;
        unsigned short month;
        unsigned short day;
        unsigned short hour;
        unsigned short minute;
        long double second;

        DateTime(unsigned short year, unsigned short month, unsigned short day,
                 unsigned short hour = 0, unsigned short minute = 0, long double sec = 0.0)
                : year(year), month(month), day(day),
                  hour(hour), minute(minute), second(sec) {}

        DateTime();

        JulianDay toJulianDay() const;

        static DateTime fromJulianDay(const JulianDay &julianDay);

        ModJulianDay toModJulianDay() const;

        static DateTime fromModJulianDay(const ModJulianDay &modJulianDay);

        friend std::ostream &operator<<(std::ostream &os, const DateTime &dateTime);

        bool operator==(const DateTime &rhs) const;

        bool operator!=(const DateTime &rhs) const;

    };

    struct Julian {
    public:
        unsigned int days;
        long double fracDays;

        virtual DateTime toDateTime() = 0;

    protected:

        Julian(unsigned int days, long double fracDays) : days(days), fracDays(fracDays) {}

        Julian() = default;

    public:
        friend std::ostream &operator<<(std::ostream &os, const Julian &julian) {
            os << "days: " << julian.days << " fracDays: " << julian.fracDays;
            return os;
        }

    };

    struct JulianDay : public Julian {

    public:

        JulianDay(unsigned int days, long double fracDays) : Julian(days, fracDays) {}

        JulianDay();

        DateTime toDateTime() override;

        static JulianDay fromDateTime(const DateTime &dateTime);

        bool operator==(const JulianDay &rhs) const;

        bool operator!=(const JulianDay &rhs) const;
    };

    struct ModJulianDay : public Julian {

    public:

        ModJulianDay(unsigned int days, long double fracDays) : Julian(days, fracDays) {}

        ModJulianDay();

        DateTime toDateTime() override;

        static ModJulianDay fromDateTime(const DateTime &dateTime);

        bool operator==(const ModJulianDay &rhs) const;

        bool operator!=(const ModJulianDay &rhs) const;
    };

}

#endif //SPP_DATETIME_H
