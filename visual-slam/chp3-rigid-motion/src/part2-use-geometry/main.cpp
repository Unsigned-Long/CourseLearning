#include <iomanip>
#include <iostream>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

namespace ns_chp3 {
void use_geometry() {
  Eigen::Matrix3d rot_mat = Eigen::Matrix3d::Identity();
  Eigen::AngleAxisd rot_vec(M_PI / 4, Eigen::Vector3d(0, 0, 1));
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "rot_vec:\n" << rot_vec.matrix() << std::endl;
  rot_mat = rot_vec.toRotationMatrix();

  Eigen::Vector3d v(1, 0, 0);
  std::cout << "(" << v.transpose() << ") rotated by 'angle axisd': ("
            << (rot_vec * v).transpose() << ")\n";

  std::cout << "(" << v.transpose() << ") rotated by 'rotate matrix': ("
            << (rot_mat * v).transpose() << ")\n";

  Eigen::Vector3d euler_angle = rot_mat.eulerAngles(2, 1, 0);

  std::cout << "(yaw[z] pitch[y] roll[x]): (" << euler_angle.transpose()
            << ")\n";

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.rotate(rot_vec);
  T.pretranslate(Eigen::Vector3d(1, 3, 4));
  std::cout << "transform matrix:\n" << T.matrix() << std::endl;

  std::cout << "(" << v.transpose() << ") rotated by 'transform': ("
            << (T * v).transpose() << ")\n";

  Eigen::Quaterniond q(rot_mat);
  std::cout << "Quaternion: " << q.coeffs().transpose() << std::endl;

  std::cout << "(" << v.transpose() << ") rotated by 'Quaternion': ("
            << (q * v).transpose() << ")\n";
  return;
}
}  // namespace ns_chp3

int main(int argc, char const *argv[]) {
  ns_chp3::use_geometry();
  return 0;
}
