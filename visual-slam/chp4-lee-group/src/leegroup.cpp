#include <iostream>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"

namespace ns_chp4 {
void leegroup() {
  double angle = M_PI / 2.0;
  Eigen::Vector3d axis(0, 0, 1);
  Eigen::Matrix3d rot_mat = Eigen::AngleAxisd(angle, axis).toRotationMatrix();

  Sophus::SO3d so3(rot_mat);

  std::cout << "so3.log(): " << so3.log().transpose() << std::endl;
  std::cout << "hat: \n" << Sophus::SO3d::hat(so3.log()) << std::endl;
  std::cout << "vee: "
            << Sophus::SO3d::vee(Sophus::SO3d::hat(so3.log())).transpose()
            << std::endl;
  std::cout << "exp(so3.log()): \n"
            << Sophus::SO3d::exp(so3.log()).matrix() << std::endl;

  Eigen::Matrix3d update_so3(Eigen::AngleAxisd(1E-10, axis).toRotationMatrix());

  std::cout << "update exp(so3.log()): \n"
            << (Sophus::SO3d::exp(Sophus::SO3d(update_so3).log()) *
                Sophus::SO3d::exp(so3.log()))
                   .matrix()
            << std::endl;

  Sophus::SE3d se3(rot_mat, Eigen::Vector3d(3, 4, 5));
  std::cout << "se3.log(): " << se3.log().transpose() << std::endl;
  std::cout << "hat: \n" << Sophus::SE3d::hat(se3.log()) << std::endl;
  std::cout << "vee: "
            << Sophus::SE3d::vee(Sophus::SE3d::hat(se3.log())).transpose()
            << std::endl;
  std::cout << "exp(se3.log()): \n"
            << Sophus::SE3d::exp(se3.log()).matrix() << std::endl;

  Sophus::SE3d update_se3(Eigen::AngleAxisd(1E-10, axis).toRotationMatrix(),
                          Eigen::Vector3d(3, 4, 5 + 1E-10));

  std::cout << "update exp(se3.log()): \n"
            << (Sophus::SE3d::exp(Sophus::SE3d(update_se3).log()) *
                Sophus::SE3d::exp(se3.log()))
                   .matrix()
            << std::endl;
  return;
}

}  // namespace ns_chp4

int main(int argc, char const *argv[]) {
  ns_chp4::leegroup();
  return 0;
}
