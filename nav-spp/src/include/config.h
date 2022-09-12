//
// Created by csl on 9/12/22.
//

#ifndef NAV_SPP_CONFIG_H
#define NAV_SPP_CONFIG_H

#include <ostream>
#include "string"

namespace ns_spp {
    struct Config {
    public:
        struct {
            std::string name;
            std::string e_mail;
        } author;

    public:
        static Config loadConfigure(const std::string &configPath);

        friend std::ostream &operator<<(std::ostream &os, const Config &config);

    private:

        Config() = default;

        static std::string readString(const std::string &filePath);
    };
}

#endif //NAV_SPP_CONFIG_H
