#include "gStaData.h"

namespace ns_gps {
  GSatData::GSatData(const std::string &staStrItem) {
    const auto ls = split(staStrItem, '\n');
    // line[0]
    this->PRN = ls[0].substr(0, 3);
    this->TOC.year = std::stoi(ls[0].substr(4, 4));
    this->TOC.month = std::stoi(ls[0].substr(9, 2));
    this->TOC.day = std::stoi(ls[0].substr(12, 2));
    this->TOC.hour = std::stoi(ls[0].substr(15, 2));
    this->TOC.minute = std::stoi(ls[0].substr(18, 2));
    this->TOC.second = std::stoi(ls[0].substr(21, 2));
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
    this->dotOMEGA = std::stod(ls[4].substr(61, 19));
    // line[5]
    this->IDOT = std::stod(ls[5].substr(4, 19));
    this->codeL2 = std::stod(ls[5].substr(23, 19));
    this->gpsWeek = std::stod(ls[5].substr(42, 19));
    this->flagL2P = std::stod(ls[5].substr(61, 19));
    // line[6]
    this->accu = std::stod(ls[6].substr(4, 19));
    this->health = std::stod(ls[6].substr(23, 19));
    this->TGD = std::stod(ls[6].substr(42, 19));
    this->IODC = std::stod(ls[6].substr(61, 19));
    // line[7]
    this->transTime = std::stod(ls[7].substr(4, 19));
    this->fitInterval = std::stod(ls[7].substr(23, 19));
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
       << ", 'omega': " << obj.omega << ", 'dotOMEGA': " << obj.dotOMEGA
       << ", 'IDOT': " << obj.IDOT << ", 'codeL2': " << obj.codeL2
       << ", 'gpsWeek': " << obj.gpsWeek << ", 'flagL2P': " << obj.flagL2P
       << ", 'accu': " << obj.accu << ", 'health': " << obj.health
       << ", 'TGD': " << obj.TGD << ", 'IODC': " << obj.IODC
       << ", 'transTime': " << obj.transTime
       << ", 'fitInterval': " << obj.fitInterval;
    os << '}';
    return os;
  }
} // namespace ns_gps
