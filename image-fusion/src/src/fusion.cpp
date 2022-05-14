#include "fusion.h"

namespace ns_fusion {
  void imgInfo(cv::Mat img, const std::string &imgName) {
    MC_VAR(imgName);
    MC_VAR(img.channels(), img.type());
    MC_VAR(img.size(), img.rows, img.cols);
  }

  std::vector<cv::Mat> splitMutiBandsImg(cv::Mat mbImg) {
    std::vector<cv::Mat> bands(mbImg.channels());
    cv::split(mbImg, bands.begin().base());
    return bands;
  }

  void showImg(const cv::String &winname, cv::Mat mat) {
    cv::namedWindow(winname, cv::WINDOW_FREERATIO);
    cv::imshow(winname, mat);
    return;
  }

  double mean(cv::Mat img) {
    CV_Assert(img.channels() == 1 && img.type() == CV_8UC1);
    double mean = 0.0;
    int rows = img.rows, cols = img.cols;
    for (int i = 0; i != rows; ++i) {
      auto ptr = img.ptr<uchar>(i);
      for (int j = 0; j != cols; ++j) {
        mean += (double)(ptr[j]);
      }
    }
    mean /= (rows * cols);
    return mean;
  }

  double covariance(cv::Mat img1, double mean1, cv::Mat img2, double mean2) {
    CV_Assert(img1.channels() == 1 && img1.type() == CV_8UC1);
    CV_Assert(img2.channels() == 1 && img2.type() == CV_8UC1);
    CV_Assert(img1.size() == img2.size());
    int rows = img1.rows, cols = img1.cols;
    double sigma12 = 0.0;
    for (int i = 0; i != rows; ++i) {
      auto ptr1 = img1.ptr<uchar>(i);
      auto ptr2 = img2.ptr<uchar>(i);
      for (int j = 0; j != cols; ++j) {
        sigma12 += ((double)(ptr1[j]) - mean1) * ((double)(ptr2[j]) - mean2);
      }
    }
    sigma12 /= (rows * cols - 1);
    return sigma12;
  }

  double corRelCoeff(cv::Mat img1, cv::Mat img2) {
    CV_Assert(img1.channels() == 1 && img1.type() == CV_8UC1);
    CV_Assert(img2.channels() == 1 && img2.type() == CV_8UC1);
    CV_Assert(img1.size() == img2.size());
    double mean1 = mean(img1), cov1 = covariance(img1, mean1, img1, mean1);
    double mean2 = mean(img2), cov2 = covariance(img2, mean2, img2, mean2);
    double cov12 = covariance(img1, mean1, img2, mean2);
    return cov12 / std::sqrt(cov1 * cov2);
  }

  std::ostream &operator<<(std::ostream &os, const Fusion::Methods &obj) {
    switch (obj) {
    case Fusion::Methods::RATIO_TRANS:
      os << "RATIO_TRANS";
      break;
    case Fusion::Methods::PRODUCT_TRANS:
      os << "PRODUCT_TRANS";
      break;
    case Fusion::Methods::WEIGHTED_FUSION:
      os << "WEIGHTED_FUSION";
      break;
    }
    return os;
  };

  void Fusion::ratioTrans(cv::Mat pImg, cv::Mat b1Img, cv::Mat b2Img, cv::Mat b3Img, cv::Mat *dstImg) {
    int rows = pImg.rows, cols = pImg.cols;
    if (dstImg->empty() || dstImg->size() != pImg.size() || dstImg->type() != CV_8UC3) {
      dstImg->create(rows, cols, CV_8UC3);
    }
    int channels = dstImg->channels();
    for (int i = 0; i != rows; ++i) {
      auto pPtr = pImg.ptr<uchar>(i);
      auto b1Ptr = b1Img.ptr<uchar>(i);
      auto b2Ptr = b2Img.ptr<uchar>(i);
      auto b3Ptr = b3Img.ptr<uchar>(i);
      auto dstPtr = dstImg->ptr<uchar>(i);
      for (int j = 0; j != cols; ++j) {
        uchar b1 = b1Ptr[j], b2 = b2Ptr[j], b3 = b3Ptr[j], p = pPtr[j];
        double factor = static_cast<double>(p) / (b1 + b2 + b3);
        dstPtr[j * channels + 0] = (uchar)(b1 * factor);
        dstPtr[j * channels + 1] = (uchar)(b2 * factor);
        dstPtr[j * channels + 2] = (uchar)(b3 * factor);
      }
    }
    return;
  }

  void Fusion::productTrans(cv::Mat pImg, cv::Mat b1Img, cv::Mat b2Img, cv::Mat b3Img, cv::Mat *dstImg) {
    int rows = pImg.rows, cols = pImg.cols;
    if (dstImg->empty() || dstImg->size() != pImg.size() || dstImg->type() != CV_8UC3) {
      dstImg->create(rows, cols, CV_8UC3);
    }
    int channels = dstImg->channels();
    int maxVal = 0, minVal = 255;
    for (int i = 0; i != rows; ++i) {
      auto pPtr = pImg.ptr<uchar>(i);
      auto b1Ptr = b1Img.ptr<uchar>(i);
      auto b2Ptr = b2Img.ptr<uchar>(i);
      auto b3Ptr = b3Img.ptr<uchar>(i);
      auto dstPtr = dstImg->ptr<uchar>(i);
      for (int j = 0; j != cols; ++j) {
        uchar b1 = b1Ptr[j], b2 = b2Ptr[j], b3 = b3Ptr[j], p = pPtr[j];
        int nb1 = (int)(b1)*p;
        int nb2 = (int)(b2)*p;
        int nb3 = (int)(b3)*p;
        auto minmax = std::minmax({nb1, nb2, nb3});
        if (minmax.first < minVal) {
          minVal = minmax.first;
        }
        if (minmax.second > maxVal) {
          maxVal = minmax.second;
        }
      }
    }
    double factor = 255.0f / (maxVal - minVal);
    for (int i = 0; i != rows; ++i) {
      auto pPtr = pImg.ptr<uchar>(i);
      auto b1Ptr = b1Img.ptr<uchar>(i);
      auto b2Ptr = b2Img.ptr<uchar>(i);
      auto b3Ptr = b3Img.ptr<uchar>(i);
      auto dstPtr = dstImg->ptr<uchar>(i);
      for (int j = 0; j != cols; ++j) {
        uchar b1 = b1Ptr[j], b2 = b2Ptr[j], b3 = b3Ptr[j], p = pPtr[j];
        int nb1 = (int)(b1)*p;
        int nb2 = (int)(b2)*p;
        int nb3 = (int)(b3)*p;
        dstPtr[j * channels + 0] = (uchar)((int(nb1) - minVal) * factor);
        dstPtr[j * channels + 1] = (uchar)((int(nb2) - minVal) * factor);
        dstPtr[j * channels + 2] = (uchar)((int(nb3) - minVal) * factor);
      }
    }
    return;
  }

  void Fusion::weightedFusion(cv::Mat pImg, cv::Mat b1Img, cv::Mat b2Img, cv::Mat b3Img, cv::Mat *dstImg) {
    int rows = pImg.rows, cols = pImg.cols;
    if (dstImg->empty() || dstImg->size() != pImg.size() || dstImg->type() != CV_8UC3) {
      dstImg->create(rows, cols, CV_8UC3);
    }
    int channels = dstImg->channels();
    // correlation coefficient
    double pb1Rel = corRelCoeff(pImg, b1Img);
    double pb2Rel = corRelCoeff(pImg, b2Img);
    double pb3Rel = corRelCoeff(pImg, b3Img);
    MC_VAR(pb1Rel, pb2Rel, pb3Rel);
    double wpb1 = 0.5 * (1 + pb1Rel), wb1 = 0.5 * (1 - pb1Rel);
    double wpb2 = 0.5 * (1 + pb2Rel), wb2 = 0.5 * (1 - pb2Rel);
    double wpb3 = 0.5 * (1 + pb3Rel), wb3 = 0.5 * (1 - pb3Rel);
    MC_VAR(wpb1, wb1);
    MC_VAR(wpb2, wb2);
    MC_VAR(wpb3, wb3);
    for (int i = 0; i != rows; ++i) {
      auto pPtr = pImg.ptr<uchar>(i);
      auto b1Ptr = b1Img.ptr<uchar>(i);
      auto b2Ptr = b2Img.ptr<uchar>(i);
      auto b3Ptr = b3Img.ptr<uchar>(i);
      auto dstPtr = dstImg->ptr<uchar>(i);
      for (int j = 0; j != cols; ++j) {
        uchar b1 = b1Ptr[j], b2 = b2Ptr[j], b3 = b3Ptr[j], p = pPtr[j];
        dstPtr[j * channels + 0] = (uchar)(p * wpb1 + b1 * wb1);
        dstPtr[j * channels + 1] = (uchar)(p * wpb2 + b2 * wb2);
        dstPtr[j * channels + 2] = (uchar)(p * wpb3 + b3 * wb3);
      }
    }
    return;
  }
} // namespace ns_fusion
