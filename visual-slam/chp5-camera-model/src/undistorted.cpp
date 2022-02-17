#include "artwork/timer/timer.h"
#include <iostream>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
namespace ns_chp5 {

  void undistorted() {
    cv::Mat img = cv::imread("../img/distorted.png", cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
      std::cout << "image load failed" << std::endl;
      return;
    }
    std::cout << "here are the details of this image:\n";
    std::cout << "{'rows': " << img.rows
              << ", 'cols': " << img.cols
              << ", 'channels': " << img.channels()
              << ", 'type': " << img.type() << "}\n";

    double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;

    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

    cv::Mat dst = cv::Mat(img.size(), CV_8UC1, cv::Scalar(0));
    ns_timer::Timer<> timer;
    for (int i = 0; i != img.rows; ++i) {
      auto dstPtr = dst.ptr<uchar>(i);
      for (int j = 0; j != img.cols; ++j) {
        double u = j, v = i;
        double x = (u - cx) / fx, y = (v - cy) / fy;
        double r_2 = x * x + y * y, r_4 = r_2 * r_2;
        double x_dis = x * (1 + k1 * r_2 + k2 * r_4) + 2 * p1 * x * y + p2 * (r_2 + 2 * x * x);
        double y_dis = y * (1 + k1 * r_2 + k2 * r_4) + 2 * p2 * x * y + p1 * (r_2 + 2 * y * y);

        int u_dis = fx * x_dis + cx, v_dis = fy * y_dis + cy;
        if (u_dis >= 0 && u_dis < img.cols && v_dis >= 0 && v_dis < img.rows) {
          dstPtr[j] = img.ptr<uchar>(v_dis)[u_dis];
        } else {
          dstPtr[j] = 0;
        }
      }
    }

    std::cout << "here is the result " << timer.last_elapsed("cost time")
              << std::endl;

    cv::imshow("win_src", img);
    cv::imshow("win_dst", dst);
    cv::waitKey(0);

    cv::imwrite("../img/undistorted.png", dst);
    return;
  }
}
  int main(int argc, char const *argv[]) {
    ns_chp5::undistorted();
    return 0;
  }
