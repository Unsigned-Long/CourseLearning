#include "gStaData.h"

int main(int argc, char const *argv[]) {
  std::fstream file("../data/gps.o", std::ios::in);
  auto str = ns_gps::readString(file);
  file.close();
  std::cout << str << std::endl;
  ns_gps::GSatData gData(str);
  std::cout << gData << std::endl;
  return 0;
}
