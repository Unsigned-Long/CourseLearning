#include "sline.h"
#define FORMAT_VECTOR
#include "artwork/csv/csv.h"
#include "icp.h"
#include "logger/logger.h"

void test_sline_fit() {
  ns_section::SLine tl(1, 0, -5);
  auto pts = ns_geo::PointSet2d::randomGenerator(50, 0, 10, 0, 10, [&tl](const ns_geo::Point2d &p) {
    return tl.distance(p) < 0.5;
  });
  auto l_ransac = ns_section::SLine::ransac(pts);
  auto l_fit = ns_section::SLine::fit(pts);
  LOG_VAR(tl, l_ransac, l_fit);
}

void test_icp() {
  auto pc1 = ns_geo::PointSet3d::randomGenerator(20000, 0.0, 10, 0.0, 10, 0.0, 10, [](const ns_geo::Point3d &p) {
    return std::abs(p.x + p.y + p.z - 10) < 0.2;
  });
  std::default_random_engine e;
  std::normal_distribution<> n(0.0, 0.1);
  Eigen::Quaterniond r(Eigen::AngleAxisd(M_PI / 8, Eigen::Vector3d(-1, 2, 3)));
  Sophus::SE3d T21(r, Eigen::Vector3d(3, 2, 1));
  auto pc2 = ns_geo::PointSet3d(pc1.size());
  for (int i = 0; i != pc1.size(); ++i) {
    const auto &p1 = pc1[i];
    auto p2 = T21 * Eigen::Vector3d(p1.x, p1.y, p1.z) + Eigen::Vector3d(n(e), n(e), n(e));
    pc2[i] = ns_geo::Point3d(p2(0), p2(1), p2(2));
  }
  auto solve = ns_section::ICP::solve(pc1, pc2);
  LOG_VAR(T21.log().transpose());
  LOG_VAR(solve.log().transpose());
  auto result = ns_geo::PointSet3d(pc1.size());
  for (int i = 0; i != pc1.size(); ++i) {
    const auto &p1 = pc1[i];
    auto p2 = solve * Eigen::Vector3d(p1.x, p1.y, p1.z);
    result[i] = ns_geo::Point3d(p2(0), p2(1), p2(2));
  }
  result.write("../pyDrawer/result.csv", std::ios::out);
}

ns_geo::PointSet3d readXYZPts(const std::string &filename) {
  auto reader = ns_csv::CSVReader::create(filename);
  ns_geo::PointSet3d pts;
  double x, y, z;
  while (reader->readLine(' ', x, y, z)) {
    pts.push_back(ns_geo::Point3d(x, 0.0, z));
  }
  return pts;
}

int main(int argc, char const *argv[]) {
  // auto laser = readXYZPts("../qt/data/laser.xyz");
  // LOG_VAR(laser.size());
  test_icp();
  return 0;
}
