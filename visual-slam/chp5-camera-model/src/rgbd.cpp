#include "artwork/csv/csv.h"
#include "artwork/timer/timer.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/highgui.hpp"
#include "pcl-1.12/pcl/point_types.h"
#include "pcl-1.12/pcl/visualization/cloud_viewer.h"
#include <algorithm>
#include <filesystem>
#include <iostream>
#include <sophus/se3.hpp>
#include <string>
#include <vector>

namespace ns_chp5 {

  struct Item {
  private:
    /**
     * @brief the members
     */
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
    Item(const double &tx, const double &ty, const double &tz,
         const double &qx, const double &qy, const double &qz, const double &qw)
        : _tx(tx),
          _ty(ty),
          _tz(tz),
          _qx(qx),
          _qy(qy),
          _qz(qz),
          _qw(qw) {}

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
    os << "'tx': " << obj.tx()
       << ", 'ty': " << obj.ty() << ", 'tz': " << obj.tz()
       << ", 'qx': " << obj.qx() << ", 'qy': " << obj.qy()
       << ", 'qz': " << obj.qz() << ", 'qw': " << obj.qw();
    os << '}';
    return os;
  }

  /**
   * \brief a function to get all the filenames in the directory
   * \param directory the directory
   * \return the filenames in the directory
   */
  std::vector<std::string> filesInDir(const std::string &directory) {
    std::vector<std::string> files;
    for (const auto &elem : std::filesystem::directory_iterator(directory))
      if (elem.status().type() != std::filesystem::file_type::directory)
        files.push_back(std::filesystem::canonical(elem.path()).c_str());
    return files;
  }

  void rgbd() {
    auto colorFiles = ns_chp5::filesInDir("../img/color");
    std::sort(colorFiles.begin(), colorFiles.end());
    auto depthFiles = ns_chp5::filesInDir("../img/depth");
    std::sort(depthFiles.begin(), depthFiles.end());
    auto pose = CSV_READ_FILE("../pose.txt", ' ', Item, double, double, double, double, double, double, double);

    std::cout << "color files:\n";
    for (const auto &elem : colorFiles)
      std::cout << elem << std::endl;

    std::cout << "\ndepth files:\n";
    for (const auto &elem : depthFiles)
      std::cout << elem << std::endl;

    std::cout << "\npose:\n";
    for (const auto &elem : pose)
      std::cout << elem << std::endl;

    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    ns_timer::Timer<> timer;
    for (int i = 0; i != 5; ++i) {
      cv::Mat color = cv::imread(colorFiles.at(i), cv::IMREAD_UNCHANGED);
      cv::Mat depth = cv::imread(depthFiles.at(i), cv::IMREAD_UNCHANGED);
      auto &cam_pose = pose.at(i);
      for (int j = 0; j != color.rows; ++j) {
        for (int k = 0; k != color.cols; ++k) {

          auto d = depth.at<ushort>(j, k);
          if (d == 0)
            continue;

          double u = k, v = j;
          double scalar = double(d) / depthScale;
          Eigen::Vector3d p((u - cx) * scalar / fx, (v - cy) * scalar / fy, scalar);

          Eigen::Quaterniond q(cam_pose.qw(), cam_pose.qx(), cam_pose.qy(), cam_pose.qz());
          Eigen::Vector3d t(cam_pose.tx(), cam_pose.ty(), cam_pose.tz());

          Eigen::Vector3d p_w = q.toRotationMatrix() * p + t;

          auto c = color.at<cv::Vec3b>(j, k);

          cloud->push_back(pcl::PointXYZRGB(p_w(0), p_w(1), p_w(2), c[2], c[1], c[0]));
        }
      }
    }

    std::cout << timer.last_elapsed("cost time") << std::endl;

    pcl::visualization::CloudViewer viewer("win");

    while (!viewer.wasStopped()) {
      viewer.showCloud(cloud);
    }
    return;
  }
}
int main(int argc, char const *argv[]) {
  ns_chp5 ::rgbd();
  return 0;
}