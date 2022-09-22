//
// Created by csl on 9/12/22.
//

#include "config.h"
#include "yaml-cpp/yaml.h"
#include "fstream"
#include "datetime.h"
#include "coordinate.h"

std::string ns_spp::Config::Author::name = "";
std::string ns_spp::Config::Author::eMail = "";
std::string ns_spp::Config::Author::address = "";

long double ns_spp::Config::Threshold::DOUBLE_EQ = 0.0;
long double ns_spp::Config::Threshold::POSITION = 0.0;
long double ns_spp::Config::Threshold::ITERATE = 0.0;

ns_spp::ModJulianDay ns_spp::Config::TimeSystem::GPSTOrigin = ModJulianDay(BigDouble("0.0"));
ns_spp::ModJulianDay ns_spp::Config::TimeSystem::BDTOrigin = ModJulianDay(BigDouble("0.0"));

ns_spp::RefEllipsoid ns_spp::Config::RefEllipsoid::WGS1984 = ns_spp::RefEllipsoid(0.0, 0.0);
ns_spp::RefEllipsoid ns_spp::Config::RefEllipsoid::CGCS2000 = ns_spp::RefEllipsoid(0.0, 0.0);

void ns_spp::Config::loadConfigure(const std::string &configPath) {
    auto doc = YAML::LoadFile(configPath);

    // authorValue info
    auto authorValue = doc["author"];
    Config::Author::name = authorValue["name"].as<std::string>();
    Config::Author::eMail = authorValue["eMail"].as<std::string>();
    Config::Author::address = authorValue["address"].as<std::string>();

    // thresholdValue
    auto thresholdValue = doc["threshold"];
    Config::Threshold::DOUBLE_EQ = thresholdValue["DOUBLE_EQ"].as<double>();
    Config::Threshold::POSITION = thresholdValue["POSITION"].as<double>();
    Config::Threshold::ITERATE = thresholdValue["ITERATE"].as<double>();

    // time system
    auto timeSystemValue = doc["timeSystem"];
    // GPST
    auto GPSTOriginValue = timeSystemValue["GPSTOrigin"];
    auto GPSOriginDateTime =
            Gregorian(GPSTOriginValue["year"].as<int>(), GPSTOriginValue["month"].as<int>(),
                      GPSTOriginValue["day"].as<int>(), GPSTOriginValue["hour"].as<int>(),
                      GPSTOriginValue["minute"].as<int>(), GPSTOriginValue["second"].as<int>());
    Config::TimeSystem::GPSTOrigin = GPSOriginDateTime.toModJulianDay();

    // BDT
    auto BDTOriginValue = timeSystemValue["BDTOrigin"];
    auto BDOriginDateTime =
            Gregorian(BDTOriginValue["year"].as<int>(), BDTOriginValue["month"].as<int>(),
                      BDTOriginValue["day"].as<int>(), BDTOriginValue["hour"].as<int>(),
                      BDTOriginValue["minute"].as<int>(), BDTOriginValue["second"].as<int>());
    Config::TimeSystem::BDTOrigin = BDOriginDateTime.toModJulianDay();

    // RefEllipsoid
    auto RefEllipsoidValue = doc["RefEllipsoid"];
    // WGS1984
    auto WGS1984Value = RefEllipsoidValue["WGS1984"];
    Config::RefEllipsoid::WGS1984 =
            ns_spp::RefEllipsoid(WGS1984Value["a"].as<double>(),
                                 1.0 / WGS1984Value["fInv"].as<double>());
    // CGCS2000
    auto CGCS2000Value = RefEllipsoidValue["CGCS2000"];
    Config::RefEllipsoid::CGCS2000 =
            ns_spp::RefEllipsoid(CGCS2000Value["a"].as<double>(),
                                 1.0 / CGCS2000Value["fInv"].as<double>());

}

std::string ns_spp::Config::readString(const std::string &filePath) {
    std::ifstream file(filePath);
    if (!file.is_open())
        throw std::runtime_error("file open failed in 'std::string readString(std::fstream &file)'");
    file.seekg(0, std::ios::end);
    auto size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::string str(size, ' ');
    file.read(const_cast<char *>(str.c_str()), size);
    file.close();
    return str;
}
