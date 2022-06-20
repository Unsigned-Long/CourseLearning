#include "circle.h"
#define FORMAT_VECTOR
#include "logger/logger.h"

int main(int argc, char const *argv[]) {
  auto pts = ns_geo::PointSet2d::randomGenerator(30, 0.0, 4.0, 1.0, 4.0, [](const ns_geo::Point2d &p) {
    ns_geo::Point2d cen(2.0, 2.0);
    double deltaX = p.x - cen.x, deltaY = p.y - cen.y;
    double theta = std::atan2(deltaY, deltaX);
    if (deltaY < 0.0) {
      theta += 2.0 * M_PI;
    }
    auto dis = ns_geo::distance(p, cen);
    return std::abs(dis - 1.5) < 0.1 && theta > 0.0 && theta < 1.0;
  });
  auto ransac = ns_section::Circle::ransac(pts);
  auto fit = ns_section::Circle::fit(pts);
  LOG_VAR(ransac);
  {
    double error = 0.0;
    for (const auto &p : pts) {
      error += ransac.distance(p);
    }
    LOG_VAR(error);
  }
  LOG_VAR(fit);
  {
    double error = 0.0;
    for (const auto &p : pts) {
      error += fit.distance(p);
    }
    LOG_VAR(error);
  }
  return 0;
}
