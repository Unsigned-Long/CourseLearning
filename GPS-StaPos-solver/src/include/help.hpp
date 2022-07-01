#ifndef HELP_HPP
#define HELP_HPP

#include <algorithm>
#include <fstream>
#include <string>
#include <vector>

namespace ns_gps {
  static std::string readString(std::fstream &file) {
    if (!file.is_open())
      throw std::runtime_error("file open failed in 'std::string readString(std::fstream &file)'");
    file.seekg(0, std::ios::end);
    auto size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::string str(size, ' ');
    file.read(const_cast<char *>(str.c_str()), size);
    return str;
  }

  static std::vector<std::string> split(const std::string &str, char splitor, bool ignoreEmpty = true) {
    std::vector<std::string> vec;
    auto iter = str.cbegin();
    while (true) {
      auto pos = std::find(iter, str.cend(), splitor);
      auto elem = std::string(iter, pos);
      if (!(elem.empty() && ignoreEmpty)) {
        vec.push_back(elem);
      }
      if (pos == str.cend()) {
        break;
      }
      iter = ++pos;
    }
    return vec;
  }

  static double julianDay(std::size_t year, std::size_t month, std::size_t day,
                          std::size_t hour = 0, std::size_t minute = 0, double second = 0.0) {
    if (month == 1 || month == 2) {
      year -= 1;
      month += 12;
    }
    int A = int(year / 100.0);
    int B = 2 - A + int(A / 4.0);
    return int(365.25 * (year + 4716)) + int(30.6001 * (month + 1)) + day +
           ((second / 60.0 + minute) / 60.0 + hour) / 24.0 + B - 1524.5;
  }

} // namespace ns_gps

#endif