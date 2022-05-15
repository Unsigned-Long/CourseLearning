#ifndef FUSION_H
#define FUSION_H

#include "macro/macro.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

namespace ns_fusion {
  /**
   * @brief print the imformation of the image
   *
   * @param img the image
   * @param imgName the name string to describe the image
   */
  void imgInfo(cv::Mat img, const std::string &imgName = "img");

  /**
   * @brief split a muti-bands image
   *
   * @param mbImg the muti-bands image
   * @return std::vector<cv::Mat> the singal band images
   */
  std::vector<cv::Mat> splitMutiBandsImg(cv::Mat mbImg);

  /**
   * @brief show an image with free ratio window
   *
   * @param winname the window name
   * @param mat the image
   */
  void showImg(const cv::String &winname, cv::Mat mat);

  /**
   * @brief compute the mean value of an image
   *
   * @param img the image
   * @return double the mean value
   */
  double mean(cv::Mat img);

  /**
   * @brief compute the covariance value of two images
   *
   * @param img1 the first image
   * @param mean1 the mean value of the first image
   * @param img2 the second image
   * @param mean2 the mean value of the second image
   * @return double the covariance value
   */
  double covariance(cv::Mat img1, double mean1, cv::Mat img2, double mean2);

  /**
   * @brief compute the correlation coefficient of two images
   *
   * @param img1 the first image
   * @param img2 the second image
   * @return double the correlation coefficient
   */
  double corRelCoeff(cv::Mat img1, cv::Mat img2);

  struct Fusion {
  public:
    enum class Methods {
      /**
       * @brief options
       */
      RATIO_TRANS,
      PRODUCT_TRANS,
      WEIGHTED_FUSION
    };

  public:
    /**
     * @brief fusion pan image and muti-bands image through Ratio Transformation
     *
     * @param pImg the pan image
     * @param b1Img the band 1 image
     * @param b2Img the band 2 image
     * @param b3Img the band 3 image
     * @param dstImg the fusion image
     */
    static void ratioTrans(cv::Mat pImg, cv::Mat b1Img, cv::Mat b2Img, cv::Mat b3Img, cv::Mat *dstImg);

    /**
     * @brief fusion pan image and muti-bands image through product transformation
     *
     * @param pImg the pan image
     * @param b1Img the band 1 image
     * @param b2Img the band 2 image
     * @param b3Img the band 3 image
     * @param dstImg the fusion image
     */
    static void productTrans(cv::Mat pImg, cv::Mat b1Img, cv::Mat b2Img, cv::Mat b3Img, cv::Mat *dstImg);

    /**
     * @brief fusion pan image and muti-bands image through weighted fusion
     *
     * @param pImg the pan image
     * @param b1Img the band 1 image
     * @param b2Img the band 2 image
     * @param b3Img the band 3 image
     * @param dstImg the fusion image
     */
    static void weightedFusion(cv::Mat pImg, cv::Mat b1Img, cv::Mat b2Img, cv::Mat b3Img, cv::Mat *dstImg);

  public:
    static double avgGradient(cv::Mat img);

    static double entropy(cv::Mat img);
  };

  /**
   * @brief override operator '<<' for type 'FusionMethods'
   */
  std::ostream &operator<<(std::ostream &os, const Fusion::Methods &obj);
} // namespace ns_fusion

#endif