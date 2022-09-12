//
// Created by csl on 9/12/22.
//

#include "../include/config.h"
#include "rapidjson/document.h"
#include "fstream"

ns_spp::Config ns_spp::Config::loadConfigure(const std::string &configPath) {
    Config config{};
    rapidjson::Document doc;
    doc.Parse(Config::readString(configPath).c_str());

    // author info
    rapidjson::Value &author = doc["author"];
    config.author.name = author["name"].GetString();
    config.author.e_mail = author["e_mail"].GetString();

    return config;
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

std::ostream &ns_spp::operator<<(std::ostream &os, const ns_spp::Config &config) {
    os << "name: " << config.author.name << " e_mail: " << config.author.e_mail;
    return os;
}

