#include "artwork/csv/csv.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
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

  ns_csv::CSVWriter writer("../data/ekalman.csv");
  writer.writeItems(',', data.front().timeStamp(), data.front().hma(), data.front().hma());

  Eigen::Matrix3d Q = Eigen::Matrix3d::Identity();
  double R = 1.0;
  Eigen::Vector3d q = Eigen::Vector3d::Ones();
  double r = 1.0;
  double b = 0.96;
  Eigen::Matrix3d P_last = Eigen::Matrix3d::Identity();
  Eigen::Vector3d X_last_hat(6.0, 1000.0, 0.0);

  auto h = [](double t, double E, double omega, double phi) -> double {
    return E * std::sin(omega * t + phi) + 9.8;
  };

  for (int i = 1; i != data.size(); ++i) {
    Eigen::Vector3d X_cur_hat_ = X_last_hat + q;
    Eigen::Matrix3d P_cur_ = P_last + Q;
    double param_E = X_cur_hat_(0);
    double param_Omega = X_cur_hat_(1);
    double param_Phi = X_cur_hat_(2);
    double t = data.at(i - 1).timeStamp();
    Eigen::MatrixXd C(1, 3);
    C(0, 0) = std::sin(param_Omega * t + param_Phi);
    C(0, 1) = std::cos(param_Omega * t + param_Phi) * t * param_E;
    C(0, 2) = std::cos(param_Omega * t + param_Phi) * param_E;
    Eigen::Vector3d K = P_cur_ * C.transpose() / ((C * P_cur_ * C.transpose())(0, 0) + R);
    double error = data.at(i).hma() - h(t, param_E, param_Omega, param_Phi) - r;
    Eigen::Vector3d X_cur_hat = X_cur_hat_ + K * error;
    Eigen::Matrix3d P_cur = (Eigen::Matrix3d::Identity() - K * C) * P_cur_;

    double d_last = (1 - b) / (1 - std::pow(b, i));
    q = (1 - d_last) * q + d_last * (X_cur_hat - X_last_hat);
    Q = (1 - d_last) * Q + d_last * (K * error * error * K.transpose() + P_cur - P_last);
    r = (1 - d_last) * r + d_last * (error + r);
    R = (1 - d_last) * R + d_last * (error * error - (C * P_cur_ * C.transpose())(0, 0));

    P_last = P_cur;
    X_last_hat = X_cur_hat;

    param_E = X_cur_hat(0);
    param_Omega = X_cur_hat(1);
    param_Phi = X_cur_hat(2);
    t = data.at(i).timeStamp();
    writer.writeItems(',', data.at(i).timeStamp(), data.at(i).hma(),
                      h(t, param_E, param_Omega, param_Phi));
  }
  return 0;
}
