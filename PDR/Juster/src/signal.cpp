#include "artwork/csv/csv.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <queue>

struct DataItem {
private:
  /**
   * @brief the members
   */
  long _timeStamp;
  double _src;
  double _dst;

public:
  /**
   * @brief construct a new DataItem object
   */
  DataItem(const long &timeStamp, const double &src, const double &dst)
      : _timeStamp(timeStamp), _src(src), _dst(dst) {}

  inline long &timeStamp() { return this->_timeStamp; }
  inline const long &timeStamp() const { return this->_timeStamp; }

  inline double &src() { return this->_src; }
  inline const double &src() const { return this->_src; }

  inline double &dst() { return this->_dst; }
  inline const double &dst() const { return this->_dst; }
};
/**
 * @brief override operator '<<' for type 'DataItem'
 */
std::ostream &operator<<(std::ostream &os, const DataItem &obj) {
  os << '{';
  os << "'timeStamp': " << obj.timeStamp() << ", 'src': " << obj.src()
     << ", 'dst': " << obj.dst();
  os << '}';
  return os;
}

bool isPeak(double before, double cur, double after) {
  return cur > before && cur > after;
}

bool isValley(double before, double cur, double after) {
  return cur < before && cur < after;
}

int main(int argc, char const *argv[]) {
  auto data = CSV_READ_FILE("../data/hma.csv", ',', DataItem, long, double, double);
  double lastTimeStamp = data.front().timeStamp();
  ns_csv::CSVWriter writer("../data/signal.csv");
  for (int i = 1; i != data.size() - 1; ++i) {
    double before = data.at(i - 1).dst();
    double cur = data.at(i).dst();
    double after = data.at(i + 1).dst();
    if (isPeak(before, cur, after) && cur - 9.8 > 0.5) {
      double f = 1.0 / ((data.at(i).timeStamp() - lastTimeStamp) / 1000.0);
      lastTimeStamp = data.at(i).timeStamp();
      writer.writeItems(',', data.at(i).timeStamp(), data.at(i).dst(), f);
    } else {
      writer.writeItems(',', data.at(i).timeStamp(), data.at(i).dst(), 0.0);
    }
  }
  return 0;
}
