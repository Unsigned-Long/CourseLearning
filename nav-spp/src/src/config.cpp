//
// Created by csl on 9/12/22.
//

#include "../include/config.h"
#include "rapidjson/document.h"
#include "fstream"
#include "datetime.h"
#include "coordinate.h"

std::string ns_spp::Config::Author::name = "";
std::string ns_spp::Config::Author::e_mail = "";

long double ns_spp::Config::Threshold::DOUBLE_EQ = 0.0;
long double ns_spp::Config::Threshold::ITERATE = 0.0;

ns_spp::ModJulianDay ns_spp::Config::TimeSystem::GPSTOrigin = ModJulianDay(BigDouble("0.0"));
ns_spp::ModJulianDay ns_spp::Config::TimeSystem::BDTOrigin = ModJulianDay(BigDouble("0.0"));

ns_spp::RefEllipsoid ns_spp::Config::RefEllipsoid::WGS1984 = ns_spp::RefEllipsoid(0.0, 0.0);
ns_spp::RefEllipsoid ns_spp::Config::RefEllipsoid::CGCS2000 = ns_spp::RefEllipsoid(0.0, 0.0);

void ns_spp::Config::loadConfigure(const std::string &configPath) {
    rapidjson::Document doc;
    doc.Parse(Config::readString(configPath).c_str());

    // authorValue info
    rapidjson::Value &authorValue = doc["author"];
    Config::Author::name = authorValue["name"].GetString();
    Config::Author::e_mail = authorValue["e_mail"].GetString();

    // thresholdValue
    rapidjson::Value &thresholdValue = doc["threshold"];
    Config::Threshold::DOUBLE_EQ = thresholdValue["DOUBLE_EQ"].GetDouble();
    Config::Threshold::ITERATE = thresholdValue["ITERATE"].GetDouble();

    // time system
    rapidjson::Value &timeSystemValue = doc["timeSystem"];
    // GPST
    rapidjson::Value &GPSTOriginValue = timeSystemValue["GPSTOrigin"];
    auto GPSOriginDateTime =
            DateTime(GPSTOriginValue["year"].GetInt(), GPSTOriginValue["month"].GetInt(),
                     GPSTOriginValue["day"].GetInt(), GPSTOriginValue["hour"].GetInt(),
                     GPSTOriginValue["minute"].GetInt(), GPSTOriginValue["second"].GetInt());
    Config::TimeSystem::GPSTOrigin = GPSOriginDateTime.toModJulianDay();

    // BDT
    rapidjson::Value &BDTOriginValue = timeSystemValue["BDTOrigin"];
    auto BDOriginDateTime =
            DateTime(BDTOriginValue["year"].GetInt(), BDTOriginValue["month"].GetInt(),
                     BDTOriginValue["day"].GetInt(), BDTOriginValue["hour"].GetInt(),
                     BDTOriginValue["minute"].GetInt(), BDTOriginValue["second"].GetInt());
    Config::TimeSystem::BDTOrigin = BDOriginDateTime.toModJulianDay();

    // RefEllipsoid
    rapidjson::Value &RefEllipsoidValue = doc["RefEllipsoid"];
    // WGS1984
    rapidjson::Value &WGS1984Value = RefEllipsoidValue["WGS1984"];
    Config::RefEllipsoid::WGS1984 =
            ns_spp::RefEllipsoid(WGS1984Value["longRadius"].GetDouble(),
                                 1.0 / WGS1984Value["oblateness_inv"].GetDouble());
    // CGCS2000
    rapidjson::Value &CGCS2000Value = RefEllipsoidValue["CGCS2000"];
    Config::RefEllipsoid::CGCS2000 =
            ns_spp::RefEllipsoid(CGCS2000Value["longRadius"].GetDouble(),
                                 1.0 / CGCS2000Value["oblateness_inv"].GetDouble());

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
