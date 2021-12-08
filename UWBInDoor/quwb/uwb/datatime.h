#pragma once

#include <iostream>

namespace ns_dt
{
#pragma region Date
    class Date
    {
    private:
        /**
         * \attention member data
         */ 
        mutable uint _year;
        mutable uint _month;
        mutable uint _day;

    public:
        // constructors
        Date() { (*this) = Date::now(); }
        Date(uint year, uint month, uint day)
            : _year(year), _month(month), _day(day) {}

        // 'get' functions
        inline const uint &year() const { return this->_year; }
        inline const uint &month() const { return this->_month; }
        inline const uint &day() const { return this->_day; }

        // the 'time machine' to change date
        void machine(uint year, uint month, uint day) const;

        // get string express
        std::string stringExpr() const;

    public:
        // get current date
        static Date now();
    };

    // overload operator '<<' for Date
    std::ostream &operator<<(std::ostream &os, const Date &d);
#pragma endregion

#pragma region Time
    class Time
    {
    private:
        /**
         * \attention member data
         */
        mutable uint _hour;
        mutable uint _minute;
        mutable uint _second;
        mutable uint _msecond;

    public:
        // constructors
        Time() { (*this) = Time::now(); }
        Time(uint hour, uint minute, uint second, uint msecond = 1)
            : _hour(hour), _minute(minute), _second(second), _msecond(msecond) {}

        // 'get' functions
        inline const uint &hour() const { return this->_hour; }
        inline const uint &minute() const { return this->_minute; }
        inline const uint &second() const { return this->_second; }
        inline const uint &msecond() const { return this->_msecond; }

        // the 'time machine' to change time
        void machine(uint hour, uint minute, uint second, uint msecond = 0) const;

        // get string express
        std::string stringExpr() const;

    public:
        // get current time
        static Time now();
    };

    // overload operator '<<' for Time
    std::ostream &operator<<(std::ostream &os, const Time &t);
#pragma endregion

#pragma region DateTime
    class DateTime
    {
    private:
        /**
         * \attention member data
         */
        Date _date;
        Time _time;

    public:
        // constructors
        DateTime() { (*this) = DateTime::now(); }
        DateTime(Date date, Time time)
            : _date(date), _time(time) {}

        // 'get' functions
        inline const Date &date() const { return this->_date; }
        inline const Time &time() const { return this->_time; }
        
        // get string express
        std::string stringExpr() const;

    public:
        // get current date and time
        static DateTime now();
    };

    // overload operator '<<' for DateTime
    std::ostream &operator<<(std::ostream &os, const DateTime &dt);
#pragma endregion

#pragma region global

    enum class TimeUnit
    {
        YEAR,
        MONTH,
        DAY,
        HOUR,
        MINUTE,
        SECOND,
        MSECOND
    };

    /**
     * \brief from Gregorian to Julian day
     */
    double calTime2Julian(const DateTime &dt);

    /**
     * \brief from day to other time ubit
     */
    double dayTo(TimeUnit tu);

    /**
     * \brief calculate the distance from 'from' to 'to' using the time unit of 'tu'
     */
    double distance(const Date &from, const Date &to, TimeUnit tu = TimeUnit::SECOND);
    double distance(const Time &from, const Time &to, TimeUnit tu = TimeUnit::SECOND);
    double distance(const DateTime &from, const DateTime &to, TimeUnit tu = TimeUnit::SECOND);

#pragma endregion
} // namespace ns_dt
