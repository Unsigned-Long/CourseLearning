#include "opencv4/opencv2/opencv.hpp"
#include "pcl-1.12/pcl/point_types.h"
#include "pcl-1.12/pcl/visualization/cloud_viewer.h"
#include <iostream>

namespace ns_chp5 {
  void depth() {
    cv::Mat left = cv::imread("../img/left.png", CV_8UC1);
    cv::Mat right = cv::imread("../img/right.png", CV_8UC1);

    std::cout << "here are the details of images:\n";
    std::cout << "'left': {'rows': " << left.rows
              << ", 'cols': " << left.cols
              << ", 'channels': " << left.channels()
              << ", 'type': " << left.type() << "}\n";
    std::cout << "'right': {'rows': " << right.rows
              << ", 'cols': " << right.cols
              << ", 'channels': " << right.channels()
              << ", 'type': " << right.type() << "}\n";

    double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
    double b = 0.573;

    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);
    cv::Mat dis_sgbm, dis;
    sgbm->compute(left, right, dis_sgbm);
    dis_sgbm.convertTo(dis, CV_32F, 1.0 / 16.0f);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

    for (int i = 0; i != left.rows; ++i) {
      for (int j = 0; j != left.cols; ++j) {
        float d = dis.at<float>(i, j);
        if (d <= 1.0f || d >= 96.0f)
          continue;
        double u = j, v = i;
        double x = (u - cx) / fx, y = (v - cy) / fy;
        double i = d / 255.0;
        d = fx * b / d;
        pcl::PointXYZI p(x * d, y * d, d, i);
        cloud->push_back(p);
      }
    }

    pcl::visualization::CloudViewer viewer("win");

    while (!viewer.wasStopped()) {
      viewer.showCloud(cloud);
    }
    return;
  }

} // namespace ns_chp5

int main(int argc, char const *argv[]) {
  ns_chp5::depth();
  return 0;
}
