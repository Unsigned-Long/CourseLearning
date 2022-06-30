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
    int month;
    int day;
    int hour;
    int minute;
    int second;

  public:
    /**
     * @brief construct a new Date object
     */
    DateTime(const int &year, const int &month, const int &day,
             const int &hour, const int &minute, const int &second)
        : year(year), month(month), day(day),
          hour(hour), minute(minute), second(second) {}

    DateTime() = default;
  };
  /**
   * @brief override operator '<<' for type 'Date'
   */
  std::ostream &operator<<(std::ostream &os, const DateTime &obj);

} // namespace ns_gps

#endif