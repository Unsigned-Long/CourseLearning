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
    int maxVal1 = 0, minVal1 = 255;
    int maxVal2 = 0, minVal2 = 255;
    int maxVal3 = 0, minVal3 = 255;
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
        if (nb1 < minVal1) {
          minVal1 = nb1;
        } else if (nb1 > maxVal1) {
          maxVal1 = nb1;
        }
        if (nb2 < minVal2) {
          minVal2 = nb2;
        } else if (nb2 > maxVal2) {
          maxVal2 = nb2;
        }
        if (nb3 < minVal3) {
          minVal3 = nb3;
        } else if (nb3 > maxVal3) {
          maxVal3 = nb3;
        }
      }
    }
    double factor1 = 255.0f / (maxVal1 - minVal1);
    double factor2 = 255.0f / (maxVal2 - minVal2);
    double factor3 = 255.0f / (maxVal3 - minVal3);
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
        dstPtr[j * channels + 0] = (uchar)((int(nb1) - minVal1) * factor1);
        dstPtr[j * channels + 1] = (uchar)((int(nb2) - minVal2) * factor2);
        dstPtr[j * channels + 2] = (uchar)((int(nb3) - minVal3) * factor3);
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

  double Fusion::avgGradient(cv::Mat img) {
    CV_Assert(!img.empty() && img.channels() == 3);
    auto grad2 = [](const cv::Vec3b &v1, const cv::Vec3b &v2) -> double {
      return std::pow(double(v1[0]) - v2[0], 2) +
             std::pow(double(v1[1]) - v2[1], 2) +
             std::pow(double(v1[2]) - v2[2], 2);
    };
    int rows = img.rows, cols = img.cols;
    double g = 0.0;
    for (int i = 0; i != rows - 1; ++i) {
      for (int j = 0; j != cols - 1; ++j) {
        auto cp = img.at<cv::Vec3b>(i, j);
        auto rp = img.at<cv::Vec3b>(i, j + 1);
        auto lp = img.at<cv::Vec3b>(i + 1, j);
        g += std::sqrt(grad2(cp, rp) + grad2(cp, lp));
      }
    }
    g /= (rows * cols - rows - cols + 1);
    return g;
  }

  double Fusion::entropy(cv::Mat img) {
    CV_Assert(!img.empty() && img.channels() == 3);
    int rows = img.rows, cols = img.cols;
    std::vector<std::vector<std::vector<std::size_t>>>
        bins(256, std::vector<std::vector<std::size_t>>(256, std::vector<std::size_t>(256, 0)));
    for (int i = 0; i != rows; ++i) {
      for (int j = 0; j != cols; ++j) {
        auto cp = img.at<cv::Vec3b>(i, j);
        ++bins[cp[0]][cp[1]][cp[2]];
      }
    }
    double h = 0.0;
    double factor = 1.0 / (rows * cols);
    for (int i = 0; i != 255; ++i) {
      for (int j = 0; j != 255; ++j) {
        for (int k = 0; k != 255; ++k) {
          if (bins[i][j][k] == 0) {
            continue;
          }
          double p = bins[i][j][k] * factor;
          h -= p * std::log2(p);
        }
      }
    }
    return h;
  }
} // namespace ns_fusion
