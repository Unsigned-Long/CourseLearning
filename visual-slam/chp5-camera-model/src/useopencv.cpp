#include "artwork/timer/timer.h"
#include <iostream>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
namespace ns_chp5 {

  void useOpenCV() {
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
    cv::imshow("win", img);
    cv::waitKey(0);

    long long sum = 0;
    ns_timer::Timer<> timer;
    for (int i = 0; i != img.rows; ++i) {
      auto ptr = img.ptr<uchar>(i);
      for (int j = 0; j != img.cols; ++j) {
        sum += ptr[j];
      }
    }
    std::cout << "{sum value of pixels: " << sum
              << "}\n"
              << timer.last_elapsed("cost time") << std::endl;

    cv::Mat img2 = img;

    std::cout << "deep copy result: img not affected\n";
    cv::Mat img3 = img2.clone();
    img3(cv::Rect(0, 0, img.cols / 4, img.rows / 4)) = cv::Mat::zeros(cv::Size2i(img.cols / 4, img.rows / 4), CV_8UC1);
    cv::imshow("win", img);
    cv::waitKey(0);

    img2(cv::Rect(0, 0, img.cols / 4, img.rows / 4)) = cv::Mat::zeros(cv::Size2i(img.cols / 4, img.rows / 4), CV_8UC1);
    std::cout << "sample agsin result: img affected\n";
    cv::imshow("win", img);
    cv::waitKey(0);

    return;
  }
}
  int main(int argc, char const *argv[]) {
    ns_chp5::useOpenCV();
    return 0;
  }
