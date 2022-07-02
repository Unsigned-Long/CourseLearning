#include "dateTime.h"
#include "help.hpp"

namespace ns_gps {
  std::ostream &operator<<(std::ostream &os, const DateTime &obj) {
    os << '{';
    os << "'year': " << obj.year << ", 'mon': " << obj.mon
       << ", 'day': " << obj.day << ", 'hour': " << obj.hour
       << ", 'min': " << obj.min << ", 'sed': " << obj.sed;
    os << '}';
    return os;
  }

  double DateTime::julianDay() const {
    return ns_gps::julianDay(year, mon, day, hour, min, sed);
  }

  double DateTime::julianSed() const {
    return this->julianDay() * 86400.0;
  }

  double GPST::sedSinceEpoch() const {
    double epoch_JS = DateTime(1980, 1, 6, 0, 0, 0.0).julianSed();
    double cur_JS = this->julianSed();
    return cur_JS - epoch_JS;
  }

  double GPST::GPSWeek() const {
    double sed = sedSinceEpoch();
    return sed / 604800.0;
  }

  double GPST::GPSSedInWeek() const {
    double gpsw = GPSWeek();
    return (gpsw - int(gpsw)) * 604800.0;
  }

  double GPST::GPSDayInWeek() const {
    double gpsw = GPSWeek();
    return (gpsw - int(gpsw)) * 7;
  }

} // namespace ns_gps
