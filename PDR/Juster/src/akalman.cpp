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

  ns_csv::CSVWriter writer("../data/akalman.csv");
  writer.writeItems(',', data.front().timeStamp(), data.front().hma(), data.front().hma());

  double Q = 1.0;
  double R = 1.0;
  double q = 1.0;
  double r = 1.0;
  double b = 0.96;
  double P_last = 1.0;
  double X_last_hat = data.front().hma();

  for (int i = 1; i != data.size(); ++i) {
    double X_cur_hat_ = X_last_hat + q;
    double P_cur_ = P_last + Q;
    double K = P_cur_ / (P_cur_ + R);
    double error = data.at(i).hma() - X_cur_hat_ - r;
    double X_cur_hat = X_cur_hat_ + K * error;
    double P_cur = (1 - K) * P_cur_;

    double d_last = (1 - b) / (1 - std::pow(b, i));
    q = (1 - d_last) * q + d_last * (X_cur_hat - X_last_hat);
    Q = (1 - d_last) * Q + d_last * (K * error * error * K + P_cur - P_last);
    r = (1 - d_last) * r + d_last * (data.at(i).hma() - X_cur_hat_);
    R = (1 - d_last) * R + d_last * (error * error - P_cur_);

    P_last = P_cur;
    X_last_hat = X_cur_hat;
    writer.writeItems(',', data.at(i).timeStamp(), data.at(i).hma(), X_cur_hat);
  }
  return 0;
}
