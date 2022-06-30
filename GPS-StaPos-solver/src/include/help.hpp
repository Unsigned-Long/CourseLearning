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
} // namespace ns_gps

#endif