#include "sline.h"
#define FORMAT_VECTOR
#include "artwork/csv/csv.h"
#include "icp.h"
#include "logger/logger.h"

ns_geo::PointSet3d readPointsLaser(const std::string &filename) {
  auto reader = ns_csv::CSVReader::create(filename);
  ns_geo::PointSet3d pts;
  double x, y, z;
  while (reader->readLine(' ', x, y, z)) {
    pts.push_back(ns_geo::Point3d(x, y, z));
  }
  return pts;
}

ns_geo::PointSet3d readPointsStation(const std::string &filename) {
  auto reader = ns_csv::CSVReader::create(filename);
  ns_geo::PointSet3d pts;
  double x, y, z;
  std::string id, code;
  while (reader->readLine(',', id, x, y, z, code)) {
    pts.push_back(ns_geo::Point3d(x, y, z));
  }
  return pts;
}

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
  auto pc1 = readPointsLaser("../qt/data/new.xyz");
  std::default_random_engine e;
  std::normal_distribution<> n(0.0, 0.05);
  Eigen::Quaterniond r(Eigen::AngleAxisd(M_PI / 10, Eigen::Vector3d(0, 0, 1)));
  Sophus::SE3d T21(r, Eigen::Vector3d(3, 0.5, 1.5));
  auto pc2 = ns_geo::PointSet3d(pc1.size());
  for (int i = 0; i != pc1.size(); ++i) {
    const auto &p1 = pc1[i];
    Eigen::Vector3d p2 = T21 * Eigen::Vector3d(p1.x, p1.y, p1.z) + Eigen::Vector3d(n(e), n(e), n(e));
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

int main(int argc, char const *argv[]) {
  auto laser = readPointsLaser("../qt/data/new.xyz");
  for (auto &p : laser) {
    p.y = 0.0;
  }
  auto station = readPointsStation("../qt/data/2019302141103.txt");
  auto Tls = ns_section::ICP::solve(station, laser);
  LOG_VAR(Tls.log().transpose());
  auto result = ns_geo::PointSet3d(station.size());
  for (int i = 0; i != station.size(); ++i) {
    const auto &p1 = station[i];
    auto p2 = Tls * Eigen::Vector3d(p1.x, p1.y, p1.z);
    result[i] = ns_geo::Point3d(p2(0), p2(1), p2(2));
  }
  station.write("../pyDrawer/pc1.csv", std::ios::out);
  laser.write("../pyDrawer/pc2.csv", std::ios::out);
  result.write("../pyDrawer/result.csv", std::ios::out);
  return 0;
}
