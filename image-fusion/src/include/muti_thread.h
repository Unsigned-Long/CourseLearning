#ifndef MUTI_THREAD_H
#define MUTI_THREAD_H

#include "opencv2/core.hpp"
#include <iostream>
#include <thread>
#include <vector>

namespace ns_fusion {
  /**
   * @brief split an image to small grids
   *
   * @param img the source image
   * @param rows the rows of this image
   * @param cols the columns of this image
   * @param clone whether to clone this source image
   * @return std::vector<std::vector<cv::Mat>> the image grids
   */
  std::vector<std::vector<cv::Mat>> gridding(cv::Mat img, std::size_t rows, std::size_t cols, bool clone = false);

  /**
   * @brief the function type to process an image
   */
  template <typename... ArgTypes>
  using ImgProcessFun = void (*)(cv::Mat srcImg, cv::Mat dstImg, ArgTypes... argvs);

  /**
   * @brief use muti threads to process an image
   *
   * @tparam ImgProcessFun the function type to process an image
   * @tparam ArgTypes the arguements
   * @param srcImg the source image
   * @param dstImg the destination image
   * @param gridRows the rows of the grids
   * @param gridCols the cols of the grids
   * @param processFun the function to process an image
   * @param argvs the arguements
   */
  template <typename ImgProcessFun, typename... ArgTypes>
  void imgMutiProcess(cv::Mat srcImg, cv::Mat dstImg,
                      std::size_t gridRows, std::size_t gridCols,
                      ImgProcessFun processFun, ArgTypes... argvs) {
    // gridding
    auto srcGrids = ns_fusion::gridding(srcImg, gridRows, gridCols, false);
    auto dstGrids = ns_fusion::gridding(dstImg, gridRows, gridCols, false);
    // muti threads
    std::vector<std::thread> threads;
    for (int i = 0; i != gridRows; ++i) {
      for (int j = 0; j != gridRows; ++j) {
        threads.emplace_back(processFun, srcGrids[i][j], dstGrids[i][j], argvs...);
      }
    }
    for (auto &t : threads) {
      t.join();
    }
    return;
  }

} // namespace ns_fusion

#endif