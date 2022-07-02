#include "gStaData.h"
#include <iomanip>

int main(int argc, char const *argv[]) {
  std::fstream file("../data/gps.n", std::ios::in);
  auto str = ns_gps::readString(file);
  file.close();
  ns_gps::GSatData gData(str);
  std::cout << gData << std::endl;
  std::cout << std::fixed << std::setprecision(3);
  std::cout << gData.staInstantPos(ns_gps::GPST(2022, 1, 1, 0, 0, 0)) << std::endl;
  // sp3: PG10  13272.605231  12135.639084  19776.721252
  // result: {'x': 13272594.510, 'y': 12135648.170, 'z': 19776721.028}
  return 0;
}
