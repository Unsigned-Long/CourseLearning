#include <iomanip>
#include <iostream>

#include "artwork/csv/csv.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "pcl-1.12/pcl/point_types.h"
#include "pcl-1.12/pcl/visualization/pcl_visualizer.h"

namespace ns_chp3 {

struct Item {
 private:
  /**
   * @brief the members
   */
  double _time;
  double _tx;
  double _ty;
  double _tz;
  double _qx;
  double _qy;
  double _qz;
  double _qw;

 public:
  /**
   * @brief construct a new Item object
   */
  Item(const double &time, const double &tx, const double &ty, const double &tz,
       const double &qx, const double &qy, const double &qz, const double &qw)
      : _time(time),
        _tx(tx),
        _ty(ty),
        _tz(tz),
        _qx(qx),
        _qy(qy),
        _qz(qz),
        _qw(qw) {}

  inline double &time() { return this->_time; }
  inline const double &time() const { return this->_time; }

  inline double &tx() { return this->_tx; }
  inline const double &tx() const { return this->_tx; }

  inline double &ty() { return this->_ty; }
  inline const double &ty() const { return this->_ty; }

  inline double &tz() { return this->_tz; }
  inline const double &tz() const { return this->_tz; }

  inline double &qx() { return this->_qx; }
  inline const double &qx() const { return this->_qx; }

  inline double &qy() { return this->_qy; }
  inline const double &qy() const { return this->_qy; }

  inline double &qz() { return this->_qz; }
  inline const double &qz() const { return this->_qz; }

  inline double &qw() { return this->_qw; }
  inline const double &qw() const { return this->_qw; }
};
/**
 * @brief override operator '<<' for type 'Item'
 */
std::ostream &operator<<(std::ostream &os, const Item &obj) {
  os << '{';
  os << "'time': " << obj.time() << ", 'tx': " << obj.tx()
     << ", 'ty': " << obj.ty() << ", 'tz': " << obj.tz()
     << ", 'qx': " << obj.qx() << ", 'qy': " << obj.qy()
     << ", 'qz': " << obj.qz() << ", 'qw': " << obj.qw();
  os << '}';
  return os;
}

void visualization() {
  auto data =
      CSV_READ_FILE("../src/part4-visualization/data.csv", ' ', Item, double,
                    double, double, double, double, double, double, double);
  std::cout << std::fixed << std::setprecision(5);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());

  pcl::visualization::PCLVisualizer viewer("win");
  viewer.setSize(1000, 640);
  viewer.setBackgroundColor(255, 255, 255);
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud");

  for (const auto &elem : data) {
    Eigen::Vector3d pos(elem.tx(), elem.ty(), elem.tz());
    Eigen::Quaterniond q(elem.qw(), elem.qx(), elem.qy(), elem.qz());
    q.normalize();
    Eigen::Isometry3d T(q);
    T.pretranslate(pos);

    viewer.addCoordinateSystem(0.04, Eigen::Affine3f(T.cast<float>().affine()));

    cloud->push_back(pcl::PointXYZ(elem.tx(), elem.ty(), elem.tz()));
    std::cout << "{'t': " << elem.time() << ", 'pos': " << cloud->back()
              << "}\n";
  }
  viewer.addPointCloud(cloud, "cloud");
  while (!viewer.wasStopped()) {
    viewer.spin();
  }
  return;
}
}  // namespace ns_chp3

int main(int argc, char const *argv[]) {
  ::ns_chp3::visualization();
  return 0;
}
