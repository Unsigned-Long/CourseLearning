#include "dateTime.h"

namespace ns_gps {
  std::ostream &operator<<(std::ostream &os, const DateTime &obj) {
    os << '{';
    os << "'year': " << obj.year << ", 'month': " << obj.month
       << ", 'day': " << obj.day << ", 'hour': " << obj.hour
       << ", 'minute': " << obj.minute << ", 'second': " << obj.second;
    os << '}';
    return os;
  }
} // namespace ns_gps
