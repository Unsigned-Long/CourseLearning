#include "fusion.h"

namespace ns_fusion {
  void imgInfo(cv::Mat img, const std::string &imgName) {
    MC_VAR(imgName);
    MC_VAR(img.channels(), img.type());
    MC_VAR(img.size(), img.rows, img.cols);
  }
} // namespace ns_fusion
