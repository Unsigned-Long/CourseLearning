#ifndef FUSION_H
#define FUSION_H

#include "macro/macro.h"
#include "opencv2/core.hpp"

namespace ns_fusion {
  /**
   * @brief print the imformation of the image
   * 
   * @param img the image
   * @param imgName the name string to describe the image
   */
  void imgInfo(cv::Mat img, const std::string &imgName = "img");
} // namespace ns_fusion

#endif