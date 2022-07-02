#include "gStaData.h"
#include <cmath>

namespace ns_gps {
  std::ostream &operator<<(std::ostream &os, const SatPRN &obj) {
    os << '{';
    os << "'flag': " << obj.flag << ", 'id': " << obj.id;
    os << '}';
    return os;
  }

  GSatData::GSatData(const std::string &staStrItem) {
    const auto ls = split(staStrItem, '\n');
    // line[0]
    this->PRN.flag = ls[0].at(0);
    this->PRN.id = std::stoi(ls[0].substr(1, 2));
    this->TOC.year = std::stoi(ls[0].substr(4, 4));
    this->TOC.mon = std::stoi(ls[0].substr(9, 2));
    this->TOC.day = std::stoi(ls[0].substr(12, 2));
    this->TOC.hour = std::stoi(ls[0].substr(15, 2));
    this->TOC.min = std::stoi(ls[0].substr(18, 2));
    this->TOC.sed = std::stoi(ls[0].substr(21, 2));
    this->cBias = std::stod(ls[0].substr(23, 19));
    this->cDrift = std::stod(ls[0].substr(42, 19));
    this->cDriftRate = std::stod(ls[0].substr(61, 19));
    // line[1]
    this->IODE = std::stod(ls[1].substr(4, 19));
    this->Crs = std::stod(ls[1].substr(23, 19));
    this->Delta_n = std::stod(ls[1].substr(42, 19));
    this->M0 = std::stod(ls[1].substr(61, 19));
    // line[2]
    this->Cuc = std::stod(ls[2].substr(4, 19));
    this->e = std::stod(ls[2].substr(23, 19));
    this->Cus = std::stod(ls[2].substr(42, 19));
    this->sqrtA = std::stod(ls[2].substr(61, 19));
    // line[3]
    this->TOE = std::stod(ls[3].substr(4, 19));
    this->Cic = std::stod(ls[3].substr(23, 19));
    this->OMEGA0 = std::stod(ls[3].substr(42, 19));
    this->Cis = std::stod(ls[3].substr(61, 19));
    // line[4]
    this->i0 = std::stod(ls[4].substr(4, 19));
    this->Crc = std::stod(ls[4].substr(23, 19));
    this->omega = std::stod(ls[4].substr(42, 19));
    this->OMEGA_DOT = std::stod(ls[4].substr(61, 19));
    // line[5]
    this->IDOT = std::stod(ls[5].substr(4, 19));
    this->L2_CODE = std::stod(ls[5].substr(23, 19));
    this->GPSW = std::stod(ls[5].substr(42, 19));
    this->L2P_DF = std::stod(ls[5].substr(61, 19));
    // line[6]
    this->accu = std::stod(ls[6].substr(4, 19));
    this->health = std::stod(ls[6].substr(23, 19));
    this->TGD = std::stod(ls[6].substr(42, 19));
    this->IODC = std::stod(ls[6].substr(61, 19));
    // line[7]
    this->transTime = std::stod(ls[7].substr(4, 19));
    this->fitInterval = std::stod(ls[7].substr(23, 19));
  }

  Point3d GSatData::staInstantPos(GPST gpst) const {
    // step 0
    double Delta_t = (gpst.GPSWeek() * 604800.0) - (GPSW * 604800.0 + TOE);
    // step 1
    double n0 = std::sqrt(ns_param::GM) / std::pow(sqrtA, 3);
    double n = n0 + Delta_n;
    // step 2
    double M = M0 + n * Delta_t;
    // step 3
    double E = M, delta_M;
    do {
      double lastE = E;
      E = M + e * std::sin(E);
      delta_M = std::abs(E - lastE);
    } while (delta_M > 1E-8);
    // step 4
    double cosf = (std::cos(E) - e) / (1.0 - e * std::cos(E));
    double sinf = (std::sqrt(1 - e * e) * std::sin(E)) / (1.0 - e * std::cos(E));
    double f = std::atan2(std::sqrt(1 - e * e) * std::sin(E), std::cos(E) - e);
    // step 5
    double u_prime = omega + f;
    // step 6
    double cos2u_prime = std::cos(2.0 * u_prime);
    double sin2u_prime = std::sin(2.0 * u_prime);
    double delta_u = Cuc * cos2u_prime + Cus * sin2u_prime;
    double delta_r = Crc * cos2u_prime + Crs * sin2u_prime;
    double delta_i = Cic * cos2u_prime + Cis * sin2u_prime;
    // step 7
    double u = u_prime + delta_u;
    double r = sqrtA * sqrtA * (1.0 - e * std::cos(E)) + delta_r;
    double i = i0 + delta_i + IDOT * Delta_t;
    // step 8
    double x = r * std::cos(u);
    double y = r * std::sin(u);
    // step 9
    double L = OMEGA0 + OMEGA_DOT * Delta_t - ns_param::OMEGA_E * TOE;
    // step 10
    double sinL = std::sin(L), cosL = std::cos(L);
    double sini = std::sin(i), cosi = std::cos(i);
    double X = x * cosL - y * cosi * sinL;
    double Y = x * sinL + y * cosi * cosL;
    double Z = y * sini;
    return Point3d(X, Y, Z);
  }

  std::ostream &operator<<(std::ostream &os, const GSatData &obj) {
    os << '{';
    os << "'PRN': " << obj.PRN << ", 'TOC': " << obj.TOC
       << ", 'cBias': " << obj.cBias << ", 'cDrift': " << obj.cDrift
       << ", 'cDriftRate': " << obj.cDriftRate << ", 'IODE': " << obj.IODE
       << ", 'Crs': " << obj.Crs << ", 'Delta_n': " << obj.Delta_n
       << ", 'M0': " << obj.M0 << ", 'Cuc': " << obj.Cuc << ", 'e': " << obj.e
       << ", 'Cus': " << obj.Cus << ", 'sqrtA': " << obj.sqrtA
       << ", 'TOE': " << obj.TOE << ", 'Cic': " << obj.Cic
       << ", 'OMEGA0': " << obj.OMEGA0 << ", 'Cis': " << obj.Cis
       << ", 'i0': " << obj.i0 << ", 'Crc': " << obj.Crc
       << ", 'omega': " << obj.omega << ", 'OMEGA_DOT': " << obj.OMEGA_DOT
       << ", 'IDOT': " << obj.IDOT << ", 'L2_CODE': " << obj.L2_CODE
       << ", 'GPSW': " << obj.GPSW << ", 'L2P_DF': " << obj.L2P_DF
       << ", 'accu': " << obj.accu << ", 'health': " << obj.health
       << ", 'TGD': " << obj.TGD << ", 'IODC': " << obj.IODC
       << ", 'transTime': " << obj.transTime
       << ", 'fitInterval': " << obj.fitInterval;
    os << '}';
    return os;
  }
} // namespace ns_gps
