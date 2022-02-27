
#include "artwork/csv/csv.h"
#include <cmath>

struct Acce {
private:
  /**
   * @brief the members
   */
  long _timeStamp;
  double _ax;
  double _ay;
  double _az;

public:
  /**
   * @brief construct a new Acce object
   */
  Acce(const long &timeStamp, const double &ax, const double &ay, const double &az)
      : _timeStamp(timeStamp), _ax(ax), _ay(ay), _az(az) {}

  inline long &timeStamp() { return this->_timeStamp; }
  inline const long &timeStamp() const { return this->_timeStamp; }

  inline double &ax() { return this->_ax; }
  inline const double &ax() const { return this->_ax; }

  inline double &ay() { return this->_ay; }
  inline const double &ay() const { return this->_ay; }

  inline double &az() { return this->_az; }
  inline const double &az() const { return this->_az; }
};
/**
 * @brief override operator '<<' for type 'Acce'
 */
std::ostream &operator<<(std::ostream &os, const Acce &obj) {
  os << '{';
  os << "'timeStamp': " << obj.timeStamp() << ", 'ax': " << obj.ax()
     << ", 'ay': " << obj.ay() << ", 'az': " << obj.az();
  os << '}';
  return os;
}

std::vector<double> weightMoveAverage(const std::vector<double> &data, std::size_t winSize) {
  std::vector<double> result(data.cbegin(), data.cbegin() + winSize - 1);
  for (int i = winSize - 1; i != data.size(); ++i) {
    double val = 0.0;
    for (int j = i - winSize + 1; j != i + 1; ++j) {
      val += (winSize + j - i) * data.at(j);
    }
    val /= winSize * (winSize + 1) / 2;
    result.push_back(val);
  }
  return result;
}

int main(int argc, char const *argv[]) {
  auto data = CSV_READ_FILE("../data/acce.csv", ',', Acce, long, double, double, double);
  std::vector<double> totalAcce;
  for (const auto &elem : data) {
    totalAcce.push_back(std::sqrt(elem.ax() * elem.ax() + elem.ay() * elem.ay() + elem.az() * elem.az()));
  }
  const std::size_t T = 10;
  auto wma_T_2 = weightMoveAverage(totalAcce, T / 2);
  auto wma_T = weightMoveAverage(totalAcce, T);
  std::vector<double> newSeq;
  for (int i = 0; i != totalAcce.size(); ++i) {
    newSeq.push_back(wma_T_2.at(i) * 2 - wma_T.at(i));
  }
  auto hma = weightMoveAverage(newSeq, std::sqrt(T));
  ns_csv::CSVWriter writer("../data/hma.csv");
  for (int i = 0; i != hma.size(); ++i) {
    writer.writeItems(',', data.at(i).timeStamp(), totalAcce.at(i), hma.at(i));
  }
  return 0;
}
