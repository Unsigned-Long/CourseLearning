#pragma once

/**
 * @file logger.h
 * @author csl (3079625093@qq.com)
 * @version 0.1
 * @date 2022-01-27
 *
 * @copyright Copyright (c) 2022
 */

#include "fmt/color.h"
#include "fmt/format.h"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

/**
 * @brief the main message type macroes:
 * everything that overrides the operator '<<' can use the static functions.
 *
 * @param info
 * {Information; Message; real-time info Of information; Of messages;
 * Informative}
 * @param process
 * {The process of achieving a goal; The development of things, especially the
 * steps of natural change;}
 * @param warning
 * {about possible accidents, etc.; a warning, warning, etc about the punishment
 * to be suffered}
 * @param error
 * {Error; Errors; Fallacy;}
 * @param fatal
 * {Fatal; Catastrophic; Destructive; Cause failure}
 *
 *
 * @brief output format for containers in the STL.
 * [1] std::pair
 * [2] std::map std::multimap std::unordered_map std::unordered_multimap
 * [3] std::set std::multiset std::unordered_set std::unordered_multiset
 * [4] std::vector std::list std::deque std::array
 *
 * @brief the macroes to control the using of the STL containers output format
 * [1] FORMAT_MAP FORMAT_MULTIMAP FORMAT_UNORDERED_MAP FORMAT_UNORDERED_MULTIMAP
 * [2] FORMAT_SET FORMAT_UNORDERED_SET FORMAT_MULTISET FORMAT_UNORDERED_MULTISET
 * [3] FORMAT_VECTOR FORMAT_LIST FORMAT_DEQUE FORMAT_ARRAY
 * [4] FORMAT_STACK FORMAT_QUEUE
 */

namespace ns_log {
  namespace ns_priv {
    /**
     * @brief base logger
     */
    class Logger {
    protected:
      /**
       * @brief the members
       */
      std::ostream *_logerOS;

    public:
      /**
       * @brief construct a new Logger object
       */
      Logger(std::ostream *os) : _logerOS(os) {}

      virtual ~Logger() {}

      template <typename... ArgvsType>
      Logger &operator()(const std::string &desc, fmt::terminal_color color, const ArgvsType &...argvs) {
        std::stringstream stream1;
        stream1 << std::fixed << std::setprecision(3);
        this->getMessageHeader(stream1, desc, color);
        (*this->_logerOS) << stream1.str();

        std::stringstream stream2;
        Logger::__print__(stream2, argvs...);
        if (this->_logerOS == &std::cout) {
          *(this->_logerOS) << fmt::format(fmt::fg(color) | fmt::emphasis::italic, "{}", stream2.str());
        } else {
          *(this->_logerOS) << fmt::format("{}", stream2.str());
        }

        return *this;
      }

      template <typename... ArgvsType>
      Logger &plaintext(const ArgvsType &...argvs) {
        std::stringstream stream;
        Logger::__print__(stream, argvs...);
        if (this->_logerOS == &std::cout) {
          *(this->_logerOS) << fmt::format(fmt::emphasis::italic, "{}", stream.str());
        } else {
          *(this->_logerOS) << fmt::format("{}", stream.str());
        }
        return *this;
      }

      template <typename... ArgvsType>
      Logger &info(const ArgvsType &...argvs) {
        (*this)("info", fmt::terminal_color::bright_green, argvs...);
        return *this;
      }

      template <typename... ArgvsType>
      Logger &warning(const ArgvsType &...argvs) {
        (*this)("warning", fmt::terminal_color::bright_yellow, argvs...);
        return *this;
      }

      template <typename... ArgvsType>
      Logger &process(const ArgvsType &...argvs) {
        (*this)("process", fmt::terminal_color::bright_blue, argvs...);
        return *this;
      }

      template <typename... ArgvsType>
      Logger &fatal(const ArgvsType &...argvs) {
        (*this)("fatal", fmt::terminal_color::bright_magenta, argvs...);
        return *this;
      }

      template <typename... ArgvsType>
      Logger &error(const ArgvsType &...argvs) {
        (*this)("error", fmt::terminal_color::bright_red, argvs...);
        return *this;
      }

      virtual void getMessageHeader(std::ostream &os, const std::string &desc, fmt::terminal_color color) = 0;

    protected:
      Logger &__print__(std::ostream &os) {
        os << '\n';
        return *this;
      }

      template <typename ArgvType>
      Logger &__print__(std::ostream &os, const ArgvType &argv) {
        os << argv << '\n';
        return *this;
      }

      template <typename ArgvType, typename... ArgvsType>
      Logger &__print__(std::ostream &os, const ArgvType &argv, const ArgvsType &...argvs) {
        os << argv;
        Logger::__print__(os, argvs...);
        return *this;
      }

      /**
       * @brief get the time when the message is outputed
       *
       * @return int64_t
       */
      double curTime() {
        auto now = std::chrono::system_clock::now();
        return std::chrono::time_point_cast<std::chrono::duration<double>>(now)
            .time_since_epoch()
            .count();
      }

    protected:
      Logger() = delete;
      Logger(const Logger &) = delete;
      Logger(Logger &&) = delete;
      Logger &operator=(const Logger &) = delete;
      Logger &operator=(Logger &&) = delete;
    };

  } // namespace ns_priv

  class StdLogger : public ns_priv::Logger {

  public:
    StdLogger(std::ostream &os) : Logger(&os) {}

    virtual ~StdLogger() {}

    virtual void getMessageHeader(std::ostream &os, const std::string &desc, fmt::terminal_color color) override {
      os << '[' << fmt::format(fmt::fg(color) | fmt::emphasis::bold, " {} ", desc) << "]-";
      os << '[' << fmt::format(fmt::fg(color) | fmt::emphasis::italic, " {:.6f}(s) ", Logger::curTime()) << "] ";
      return;
    }
  };

  class FileLogger : public ns_priv::Logger {
  public:
    FileLogger(const std::string &filename) : Logger(new std::ofstream(filename, std::ios::out)) {}

    virtual ~FileLogger() {
      delete this->_logerOS;
    }

    virtual void getMessageHeader(std::ostream &os, const std::string &desc, fmt::terminal_color color) override {
      os << fmt::format("[ {0} ]-[ {1:.6f}(s) ] ", desc, Logger::curTime());
      return;
    }
  };

  namespace ns_priv {
    /**
     * @brief params to control
     * @param _splitor_ the splitor to split the elements
     * @param _firName_ the describe name for the first element of the std::pair
     * @param _sedName_ the describe name for the second element of the std::pair
     */
    static const std::string _splitor_(", ");

    static StdLogger stdCoutLogger(std::cout);
  } // namespace ns_priv

  /**
   * @brief the main message type macroes
   *
   * [0] plaintext {the plain text}
   * [1] info    {Information; Message; real-time info Of information; Of messages; Informative}
   * [2] process {The process of achieving a goal; The development of things, especially the steps of natural change;}
   * [3] warning {about possible accidents, etc.; a warning, warning, etc about the punishment to be suffered}
   * [4] error   {Error; Errors; Fallacy;}
   * [5] fatal   {Fatal; Catastrophic; Destructive; Cause failure}
   *
   */

  template <typename... ArgvsType>
  static ns_priv::Logger &plaintext(const ArgvsType &...argvs) {
    return ns_log::ns_priv::stdCoutLogger.plaintext(argvs...);
  }

  template <typename... ArgvsType>
  static ns_priv::Logger &info(const ArgvsType &...argvs) {
    return ns_log::ns_priv::stdCoutLogger.info(argvs...);
  }

  template <typename... ArgvsType>
  static ns_priv::Logger &process(const ArgvsType &...argvs) {
    return ns_log::ns_priv::stdCoutLogger.process(argvs...);
  }

  template <typename... ArgvsType>
  static ns_priv::Logger &warning(const ArgvsType &...argvs) {
    return ns_log::ns_priv::stdCoutLogger.warning(argvs...);
  }

  template <typename... ArgvsType>
  static ns_priv::Logger &error(const ArgvsType &...argvs) {
    return ns_log::ns_priv::stdCoutLogger.error(argvs...);
  }

  template <typename... ArgvsType>
  static ns_priv::Logger &fatal(const ArgvsType &...argvs) {
    return ns_log::ns_priv::stdCoutLogger.fatal(argvs...);
  }

} // namespace ns_log

#pragma region output for container

/**
 * @brief overload the operator '<<' for std::pair
 */
template <typename Key, typename Val>
std::ostream &operator<<(std::ostream &os, const std::pair<Key, Val> &p) {
  os << "{'" << p.first << "': " << p.second << '}';
  return os;
}

/**
 * @brief output format for container
 */
template <typename ConType>
std::ostream &orderedConer(std::ostream &os, const ConType &s) {
  os << '[';
  if (s.empty()) {
    os << "]";
    return os;
  }
  auto iter = s.cbegin();
  for (; iter != (--s.cend()); ++iter)
    os << *iter << ns_log::ns_priv::_splitor_;
  os << *iter << ']';
  return os;
}

/**
 * @brief output format for unordered container
 */
template <typename ConType>
std::ostream &unorderedConer(std::ostream &os, const ConType &c) {
  os << '[';
  if (c.empty()) {
    os << "]";
    return os;
  }
  std::stringstream stream;
  for (const auto &elem : c)
    stream << elem << ns_log::ns_priv::_splitor_;
  std::string str = stream.str();
  os << std::string_view(str.c_str(),
                         str.size() - ns_log::ns_priv::_splitor_.size())
     << ']';
  return os;
}
#pragma endregion

#pragma region map

#ifdef FORMAT_MAP
#include <map>
/**
 * @brief overload the operator '<<' for std::map
 */
template <typename Key, typename Val>
std::ostream &operator<<(std::ostream &os, const std::map<Key, Val> &m) {
  return orderedConer(os, m);
}
#endif

#ifdef FORMAT_MULTIMAP
#include <map>
/**
 * @brief overload the operator '<<' for std::multimap
 */
template <typename Key, typename Val>
std::ostream &operator<<(std::ostream &os, const std::multimap<Key, Val> &m) {
  return orderedConer(os, m);
}
#endif

#ifdef FORMAT_UNORDERED_MAP
#include <unordered_map>
/**
 * @brief overload the operator '<<' for std::unordered_map
 */
template <typename Key, typename Val>
std::ostream &operator<<(std::ostream &os,
                         const std::unordered_map<Key, Val> &m) {
  return unorderedConer(os, m);
}

#endif

#ifdef FORMAT_UNORDERED_MULTIMAP
#include <unordered_map>
/**
 * @brief overload the operator '<<' for std::unordered_multimap
 */
template <typename Key, typename Val>
std::ostream &operator<<(std::ostream &os,
                         const std::unordered_multimap<Key, Val> &m) {
  return unorderedConer(os, m);
}
#endif

#pragma endregion

#pragma region set

#ifdef FORMAT_SET
#include <set>
/**
 * @brief overload the operator '<<' for std::set
 */
template <typename Val>
std::ostream &operator<<(std::ostream &os, const std::set<Val> &s) {
  return orderedConer(os, s);
}
#endif

#ifdef FORMAT_UNORDERED_SET
#include <unordered_set>
/**
 * @brief overload the operator '<<' for std::unordered_set
 */
template <typename Val>
std::ostream &operator<<(std::ostream &os, const std::unordered_set<Val> &s) {
  return unorderedConer(os, s);
}
#endif

#ifdef FORMAT_MULTISET
#include <set>
/**
 * @brief overload the operator '<<' for std::multiset
 */
template <typename Val>
std::ostream &operator<<(std::ostream &os, const std::multiset<Val> &s) {
  return orderedConer(os, s);
}
#endif

#ifdef FORMAT_UNORDERED_MULTISET
#include <unordered_set>
/**
 * @brief overload the operator '<<' for std::unordered_multiset
 */
template <typename Val>
std::ostream &operator<<(std::ostream &os,
                         const std::unordered_multiset<Val> &s) {
  return unorderedConer(os, s);
}
#endif
#pragma endregion

#pragma region seq cons

#ifdef FORMAT_VECTOR
#include <vector>
/**
 * @brief overload the operator '<<' for std::vector
 */
template <typename Val>
std::ostream &operator<<(std::ostream &os, const std::vector<Val> &s) {
  return orderedConer(os, s);
}
#endif

#ifdef FORMAT_LIST
#include <list>
/**
 * @brief overload the operator '<<' for std::list
 */
template <typename Val>
std::ostream &operator<<(std::ostream &os, const std::list<Val> &s) {
  return orderedConer(os, s);
}
#endif

#ifdef FORMAT_DEQUE
#include <deque>
/**
 * @brief overload the operator '<<' for std::deque
 */
template <typename Val>
std::ostream &operator<<(std::ostream &os, const std::deque<Val> &s) {
  return orderedConer(os, s);
}
#endif

#ifdef FORMAT_ARRAY
#include <array>
/**
 * @brief overload the operator '<<' for std::array
 */
template <typename Val, std::size_t Size>
std::ostream &operator<<(std::ostream &os, const std::array<Val, Size> &s) {
  os << '[';
  for (int i = 0; i != s.size() - 1; ++i)
    os << s[i] << ns_log::ns_priv::_splitor_;
  os << s.back() << ']';
  return os;
}
#endif

#ifdef FORMAT_STACK
#include <stack>
/**
 * @brief overload the operator '<<' for std::stack
 */
template <typename Val>
std::ostream &operator<<(std::ostream &os, const std::stack<Val> &s) {
  if (s.empty()) {
    os << "[]";
    return os;
  }
  os << "['top' ";
  auto cs = s;
  while (cs.size() != 1) {
    os << cs.top() << ns_log::ns_priv::_splitor_;
    cs.pop();
  }
  os << cs.top() << "]";
  return os;
}
#endif

#ifdef FORMAT_QUEUE
#include <queue>
/**
 * @brief overload the operator '<<' for std::queue
 */
template <typename Val>
std::ostream &operator<<(std::ostream &os, const std::queue<Val> &q) {
  if (q.empty()) {
    os << "[]";
    return os;
  }
  os << "['front' ";
  auto cq = q;
  while (cq.size() != 1) {
    os << cq.front() << ns_log::ns_priv::_splitor_;
    cq.pop();
  }
  os << cq.front() << "]";
  return os;
}
#endif

#pragma endregion
