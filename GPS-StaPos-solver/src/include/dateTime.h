#ifndef DATETIME_H
#define DATETIME_H

#include <iostream>
#include <string>

namespace ns_gps {

  struct DateTime {
  public:
    /**
     * @brief the members
     */
    int year;
    int mon;
    int day;
    int hour;
    int min;
    int sed;

  public:
    /**
     * @brief construct a new Date object
     */
    DateTime(const int &year, const int &month, const int &day,
             const int &hour, const int &minute, const int &second)
        : year(year), mon(month), day(day),
          hour(hour), min(minute), sed(second) {}

    DateTime() = default;

    double julianDay() const;

    double julianSed() const;
  };
  /**
   * @brief override operator '<<' for type 'Date'
   */
  std::ostream &operator<<(std::ostream &os, const DateTime &obj);

  struct GPST : public DateTime {
  public:
    using DateTime::DateTime;

    double sedSinceEpoch() const;

    double GPSWeek() const;

    double GPSSedInWeek() const;

    double GPSDayInWeek() const;
  };

} // namespace ns_gps

#endif