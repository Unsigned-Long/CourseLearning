#include "flags/flags.hpp"
#include "fusion.h"
#include "logger/logger.h"
#include "muti_thread.h"
#include "timer/timer.h"
#include <iostream>

const std::size_t gridRows = 8, gridCols = 8;

int main(int argc, char const *argv[]) {
  try {

    // prepare for parmaeters
    std::string mutiBandsImgName = "../img/DOM-100CM-ROI.tif";
    std::string panImgName = "../img/DOM-5CM-ROI-FPAN.tif";
    std::string outputImgName = "../img/fusion.tif";
    ns_timer::Timer timer;
    ns_flags::OptionParser parser;

    parser.addOption<ns_flags::IntArg>("method", 0,
                                       "the method to fusion images [RATIO_TRANS(0), PRODUCT_TRANS(1), WEIGHTED_FUSION(2)]",
                                       ns_flags::OptionProp::REQUIRED);
    parser.addOption<ns_flags::StringArg>("mbImg", mutiBandsImgName,
                                          "the name of the muti-bands image",
                                          ns_flags::OptionProp::OPTIONAL);
    parser.addOption<ns_flags::StringArg>("pImg", panImgName,
                                          "the name of the pan image",
                                          ns_flags::OptionProp::OPTIONAL);
    parser.addOption<ns_flags::StringArg>("oImg", outputImgName,
                                          "the name of the output image",
                                          ns_flags::OptionProp::OPTIONAL);
    parser.setupParser(argc, argv);
    int m = parser.getOptionArgv<ns_flags::IntArg>("method");
    ns_fusion::Fusion::Methods method;
    if (m == 0) {
      method = ns_fusion::Fusion::Methods::RATIO_TRANS;
    } else if (m == 1) {
      method = ns_fusion::Fusion::Methods::PRODUCT_TRANS;
    } else if (m == 2) {
      method = ns_fusion::Fusion::Methods::WEIGHTED_FUSION;
    } else {
      ns_log::error("the value of option 'method' is invalid, please input: [RATIO_TRANS(0), PRODUCT_TRANS(1), WEIGHTED_FUSION(2)]");
      return -1;
    }

    // get parameters
    mutiBandsImgName = parser.getOptionArgv<ns_flags::StringArg>("mbImg");
    panImgName = parser.getOptionArgv<ns_flags::StringArg>("pImg");
    outputImgName = parser.getOptionArgv<ns_flags::StringArg>("oImg");
    MC_VAR(mutiBandsImgName);
    MC_VAR(panImgName);
    MC_VAR(method);
    MC_VAR(outputImgName);

    // load images
    ns_log::info("load images...");
    timer.re_start();
    auto mbImg = cv::imread(mutiBandsImgName, cv::IMREAD_UNCHANGED);
    if (mbImg.empty()) {
      ns_log::error("loading muti-bands image from \"", mutiBandsImgName, "\" failed.");
      return -1;
    }
    ns_fusion::imgInfo(mbImg, "muti-bands image");
    MC_ENDL;
    auto pImg = cv::imread(panImgName, cv::IMREAD_UNCHANGED);
    if (mbImg.empty()) {
      ns_log::error("loading pan image from \"", panImgName, "\" failed.");
      return -1;
    }
    ns_fusion::imgInfo(pImg, "pan image");
    ns_log::plaintext(timer.last_elapsed("load image cost"));
    // display
    ns_log::info("load images finished, display images(waiting key)...");
    timer.re_start();
    ns_fusion::showImg(mutiBandsImgName, mbImg);
    ns_fusion::showImg(panImgName, pImg);
    cv::waitKey(0);
    cv::destroyAllWindows();
    ns_log::plaintext(timer.last_elapsed("display image cost"));
    // resize
    if (mbImg.size() != pImg.size()) {
      timer.re_start();
      ns_log::process("resize the muti-bands image...");
      cv::resize(mbImg, mbImg, cv::Size(pImg.cols, pImg.rows));
      ns_fusion::imgInfo(mbImg, "muti-bands image");
      ns_log::plaintext(timer.last_elapsed("resize muti-bands image cost"));
    }
    // split
    ns_log::process("split the muti-bands image...");
    timer.re_start();
    std::vector<cv::Mat> bands = ns_fusion::splitMutiBandsImg(mbImg);
    for (int i = 0; i != bands.size(); ++i) {
      std::string winName = "band-" + std::to_string(i + 1);
      ns_fusion::showImg(winName, bands.at(i));
      ns_fusion::imgInfo(bands.at(i), winName);
      MC_ENDL;
    }
    ns_log::plaintext(timer.last_elapsed("split muti-bands image cost"));
    cv::waitKey(0);
    cv::destroyAllWindows();
    // process
    timer.re_start();
    cv::Mat dst;
    if (method == ns_fusion::Fusion::Methods::RATIO_TRANS) {
      // Ratio Transformation
      ns_log::process("Ratio Transformation...");
      ns_fusion::Fusion::ratioTrans(pImg, bands[0], bands[1], bands[2], &dst);
      ns_fusion::showImg("Ratio Transformation", dst);
    } else if (method == ns_fusion::Fusion::Methods::PRODUCT_TRANS) {
      // Product Transformation
      ns_log::process("Product Transformation...");
      ns_fusion::Fusion::productTrans(pImg, bands[0], bands[1], bands[2], &dst);
      ns_fusion::showImg("Product Transformation", dst);
    } else if (method == ns_fusion::Fusion::Methods::WEIGHTED_FUSION) {
      // Weighted Fusion
      ns_log::process("Weighted Fusion...");
      ns_fusion::Fusion::weightedFusion(pImg, bands[0], bands[1], bands[2], &dst);
      ns_fusion::showImg("Weighted Fusion", dst);
    }
    ns_log::plaintext(timer.last_elapsed("cost"));
    cv::waitKey(0);
    cv::destroyAllWindows();
    // write
    ns_log::info("writing image");
    timer.re_start();
    cv::imwrite(outputImgName, dst);
    ns_log::plaintext(timer.last_elapsed("cost"));
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
  }

  return 0;
}
