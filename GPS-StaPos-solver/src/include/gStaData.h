#ifndef GSTADATA_H
#define GSTADATA_H

#include "dateTime.h"
#include "help.hpp"
#include "point3d.h"
#include <iostream>
#include <string>

namespace ns_gps {

  namespace ns_param {
    static constexpr double GM = 3.986005E14;
    static constexpr double OMEGA_E = 7.292115E-5;
  } // namespace ns_param

  struct SatPRN {
  public:
    /**
     * @brief the members
     */
    char flag;
    std::size_t id;

  public:
    /**
     * @brief construct a new SatPRN object
     */
    SatPRN(const char &flag, const std::size_t &id) : flag(flag), id(id) {}

    SatPRN() = default;
  };
  /**
   * @brief override operator '<<' for type 'SatPRN'
   */
  std::ostream &operator<<(std::ostream &os, const SatPRN &obj);

  struct GSatData {
  public:
    /**
     * @brief the members
     */
    // Satellite system (G), sat number (PRN)
    SatPRN PRN;
    // Epoch: Toc - Time of Clock (GPS)
    GPST TOC;
    // SV clock bias (seconds)
    double cBias;
    // SV clock drift (sec/sec)
    double cDrift;
    // SV clock drift rate (sec/sec2)
    double cDriftRate;
    // IODE Issue of Data, Ephemeris
    double IODE;
    // Crs (meters)
    double Crs;
    // Delta n (radians/sec)
    double Delta_n;
    // M0 (radians)
    double M0;
    // Cuc (radians)
    double Cuc;
    // e Eccentricity
    double e;
    // Cus (radians)
    double Cus;
    // sqrt(A) (sqrt(m))
    double sqrtA;
    // Toe Time of Ephemeris (sec of GPS week)
    double TOE;
    // Cic (radians)
    double Cic;
    // OMEGA0 (radians)
    double OMEGA0;
    // Cis (radians)
    double Cis;
    // i0 (radians)
    double i0;
    // Crc (meters)
    double Crc;
    // omega (radians)
    double omega;
    // OMEGA DOT (radians/sec)
    double OMEGA_DOT;
    // IDOT (radians/sec)
    double IDOT;
    // Codes on L2 channel
    double L2_CODE;
    // GPS Week
    double GPSW;
    // L2 P data flag
    double L2P_DF;
    // SV accuracy (meters)
    double accu;
    // SV health
    double health;
    // TGD (seconds)
    double TGD;
    // IODC Issue of Data, Clock
    double IODC;
    // Transmission time of message
    double transTime;
    // Fit Interval in hours
    double fitInterval;

  public:
    /**
     * @brief construct a new GSatData object
     */
    GSatData(const std::string &staStrItem);

    Point3d staInstantPos(GPST gpst) const;
  };
  /**
   * @brief override operator '<<' for type 'GSatData'
   */
  std::ostream &operator<<(std::ostream &os, const GSatData &obj);

} // namespace ns_gps

#endif