#include "sline.h"
#define FORMAT_VECTOR
#include "logger/logger.h"

int main(int argc, char const *argv[]) {
  ns_section::SLine tl(1, 0, -5);
  auto pts = ns_geo::PointSet2d::randomGenerator(50, 0, 10, 0, 10, [&tl](const ns_geo::Point2d &p) {
    return tl.distance(p) < 0.5;
  });
  auto l_ransac = ns_section::SLine::ransac(pts);
  auto l_fit = ns_section::SLine::fit(pts);
  LOG_VAR(tl, l_ransac, l_fit);
  return 0;
}
