#include <iostream>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

namespace ns_chp3 {
void exp_case() {
  Eigen::Quaterniond q1(0.35, 0.2, 0.3, 0.1);
  q1.normalize();
  Eigen::Vector3d t1(0.3, 0.1, 0.1);
  Eigen::Isometry3d T1(q1);
  T1.pretranslate(t1);

  Eigen::Quaterniond q2(-0.5, 0.4, -0.1, 0.2);
  q2.normalize();
  Eigen::Vector3d t2(-0.1, 0.5, 0.3);
  Eigen::Isometry3d T2(q2);
  T2.pretranslate(t2);

  Eigen::Vector3d p(0.5, 0, 0.2);
  Eigen::Vector3d result = T2 * T1.inverse() * p;

  std::cout << "the result is: " << result.transpose() << std::endl;
  return;
}

}  // namespace ns_chp3

int main(int argc, char const *argv[]) {
  ns_chp3::exp_case();
  return 0;
}
