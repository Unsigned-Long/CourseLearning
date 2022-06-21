#include "sline.h"
#define FORMAT_VECTOR
#include "logger/logger.h"

int main(int argc, char const *argv[]) {
  ns_section::SLine tl(0, 1, -5);
  auto pts = ns_geo::PointSet2d::randomGenerator(50, 0, 10, 0, 10, [&tl](const ns_geo::Point2d &p) {
    return tl.distance(p) < 0.5;
  });
  auto l_fit = ns_section::SLine::fit(pts);
  LOG_VAR(tl, l_fit);
  float error_truth = 0.0;
  float error_fit = 0.0;
  for (const auto &p : pts) {
    error_truth += tl.distance(p);
    error_fit += l_fit.distance(p);
  }
  LOG_VAR(error_truth, error_fit);
  return 0;
}
