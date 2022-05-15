#include "flags/flags.hpp"
#include "fusion.h"
#include "logger/logger.h"
#include "timer/timer.h"
#include <iostream>

int main(int argc, char const *argv[]) {
  try {
    ns_timer::Timer timer;
    ns_flags::OptionParser parser;
    parser.addOption<ns_flags::StringVecArg>("imgs", {}, "the name list of images to be evaluated", ns_flags::OptionProp::REQUIRED);
    parser.setupParser(argc, argv);
    auto imgNames = parser.getOptionArgv<ns_flags::StringVecArg>("imgs");
    for (const auto &name : imgNames) {
      cv::Mat img = cv::imread(name, cv::IMREAD_UNCHANGED);
      if (img.empty()) {
        ns_log::error("loading image from \"", name, "\" failed.");
        return -1;
      }
      ns_log::process("process new image [", name, ']');
      timer.re_start();
      ns_fusion::imgInfo(img, name);
      auto avgGradient = ns_fusion::Fusion::avgGradient(img);
      auto entropy = ns_fusion::Fusion::entropy(img);
      MC_VAR(avgGradient, entropy);
      ns_log::plaintext(timer.last_elapsed("cost"));
      MC_ENDL;
    }
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
  }

  return 0;
}
