#include "artwork/csv/csv.h"
#include <cmath>
#include <iostream>

struct DataItem {
private:
  /**
   * @brief the members
   */
  long _timeStamp;
  double _totalAcce;
  double _hma;

public:
  /**
   * @brief construct a new DataItem object
   */
  DataItem(const long &timeStamp, const double &totalAcce, const double &hma)
      : _timeStamp(timeStamp), _totalAcce(totalAcce), _hma(hma) {}

  inline long &timeStamp() { return this->_timeStamp; }
  inline const long &timeStamp() const { return this->_timeStamp; }

  inline double &totalAcce() { return this->_totalAcce; }
  inline const double &totalAcce() const { return this->_totalAcce; }

  inline double &hma() { return this->_hma; }
  inline const double &hma() const { return this->_hma; }
};
/**
 * @brief override operator '<<' for type 'DataItem'
 */
std::ostream &operator<<(std::ostream &os, const DataItem &obj) {
  os << '{';
  os << "'timeStamp': " << obj.timeStamp()
     << ", 'totalAcce': " << obj.totalAcce() << ", 'hma': " << obj.hma();
  os << '}';
  return os;
}
int main(int argc, char const *argv[]) {
  auto data = CSV_READ_FILE("../data/hma.csv", ',', DataItem, long, double, double);

  ns_csv::CSVWriter writer("../data/kalman.csv");
  writer.writeItems(',', data.front().timeStamp(), data.front().hma(), data.front().hma());

  double Q = 1;
  double R = 0.01;
  double P_last = 0.1;
  double X_last = data.front().hma();

  for (int i = 1; i != data.size(); ++i) {
    double X_pred = 2 * 9.8 - X_last;
    double P_cur = P_last + Q;

    double K_cur = P_cur * (P_cur + R);

    X_last = X_pred + K_cur * (data.at(i).hma() - X_pred);
    P_last = (1.0 - K_cur) * P_cur;
    writer.writeItems(',', data.at(i).timeStamp(), data.at(i).hma(), X_last);
  }
  return 0;
}
