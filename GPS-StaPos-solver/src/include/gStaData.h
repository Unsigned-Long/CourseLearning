#ifndef GSTADATA_H
#define GSTADATA_H

#include "dateTime.h"
#include "help.hpp"
#include <iostream>
#include <string>

namespace ns_gps {

  struct GSatData {
  public:
    /**
     * @brief the members
     */
    std::string PRN;
    DateTime TOC;
    double cBias;
    double cDrift;
    double cDriftRate;
    double IODE;
    double Crs;
    double Delta_n;
    double M0;
    double Cuc;
    double e;
    double Cus;
    double sqrtA;
    double TOE;
    double Cic;
    double OMEGA0;
    double Cis;
    double i0;
    double Crc;
    double omega;
    double dotOMEGA;
    double IDOT;
    double codeL2;
    double gpsWeek;
    double flagL2P;
    double accu;
    double health;
    double TGD;
    double IODC;
    double transTime;
    double fitInterval;

  public:
    /**
     * @brief construct a new GSatData object
     */
    GSatData(const std::string &staStrItem);
  };
  /**
   * @brief override operator '<<' for type 'GSatData'
   */
  std::ostream &operator<<(std::ostream &os, const GSatData &obj);

} // namespace ns_gps

#endif