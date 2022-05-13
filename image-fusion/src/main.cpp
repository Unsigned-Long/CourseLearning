#include "fusion.h"
#include "muti_thread.h"
#include "opencv2/highgui.hpp"
#include "timer/timer.h"
#include <iostream>

const std::size_t gridRows = 8, gridCols = 8;

void process(cv::Mat src, cv::Mat dst) {
  int rows = src.rows, cols = src.cols, channels = src.channels();

  for (int i = 0; i != rows; ++i) {
    auto srcPtr = src.ptr<uchar>(i);
    auto dstPtr = dst.ptr<uchar>(i);
    for (int j = 0; j != cols; ++j) {
      dstPtr[j * channels + 0] = dstPtr[j * channels + 1] = dstPtr[j * channels + 2] = srcPtr[j * channels + 0];
      std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
  }

  return;
}

int main(int argc, char const *argv[]) {
  cv::Mat img = cv::imread("../img/dlam.jpg", cv::IMREAD_UNCHANGED);
  ns_fusion::imgInfo(img, "testImg");

  // test muti thread
  ns_timer::Timer timer;
  timer.re_start();
  cv::Mat result(img.rows, img.cols, img.type());
  process(img, result);
  std::cout << timer.last_elapsed("no muti thread") << std::endl;
  timer.re_start();
  ns_fusion::imgMutiProcess(img, result, gridRows, gridCols, process);
  std::cout << timer.last_elapsed("muti thread") << std::endl;

  // image display
  cv::imshow("src", img);
  cv::imshow("dst", result);
  cv::waitKey(0);
  return 0;
}
